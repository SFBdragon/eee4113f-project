use crate::protocol::LoRaAddr;
use rusqlite::{Connection, OptionalExtension, params};
use std::path::PathBuf;

// ── Database handle ───────────────────────────────────────────────────────────

pub struct Database {
    conn: Connection,
}

impl Database {
    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /// Open (or create) the database next to the running executable.
    pub fn open() -> Result<Self, rusqlite::Error> {
        let path = db_path();
        Self::open_at(path)
    }

    /// Open at an explicit path. Useful for tests.
    pub fn open_at(path: impl AsRef<std::path::Path>) -> Result<Self, rusqlite::Error> {
        let conn = Connection::open(path)?;
        let db = Self { conn };
        db.configure()?;
        db.migrate()?;
        Ok(db)
    }

    fn configure(&self) -> Result<(), rusqlite::Error> {
        // WAL mode: better concurrent read performance; safe for single-process multi-thread use.
        self.conn.execute_batch(
            "
            PRAGMA journal_mode = WAL;
            PRAGMA synchronous = NORMAL;
            PRAGMA foreign_keys = ON;
        ",
        )?;
        Ok(())
    }

    fn migrate(&self) -> Result<(), rusqlite::Error> {
        self.conn.execute_batch(
            "
            CREATE TABLE IF NOT EXISTS addresses (
                address INTEGER PRIMARY KEY
            );

            CREATE TABLE IF NOT EXISTS blocks (
                block_id INTEGER NOT NULL,
                address INTEGER NOT NULL REFERENCES addresses(address),
                data BLOB NOT NULL,
                PRIMARY KEY (block_id, address)
            );

            CREATE INDEX IF NOT EXISTS idx_blocks_address_blockid
                ON blocks (address, block_id);
        ",
        )?;
        Ok(())
    }

    // ── Address management ────────────────────────────────────────────────────

    pub fn add_address(&self, address: LoRaAddr) -> Result<(), rusqlite::Error> {
        self.conn.execute(
            "INSERT OR IGNORE INTO addresses (address) VALUES (?1)",
            params![address.to_raw() as i64],
        )?;
        Ok(())
    }

    pub fn remove_address(&self, address: LoRaAddr) -> Result<(), rusqlite::Error> {
        // Cascading deletes are not automatic without ON DELETE CASCADE in schema;
        // delete blocks explicitly first.
        self.conn.execute(
            "DELETE FROM blocks  WHERE address = ?1",
            params![address.to_raw() as i64],
        )?;
        self.conn.execute(
            "DELETE FROM addresses WHERE address = ?1",
            params![address.to_raw() as i64],
        )?;
        Ok(())
    }

    pub fn list_addresses(&self) -> Result<Vec<LoRaAddr>, rusqlite::Error> {
        let mut stmt = self
            .conn
            .prepare("SELECT address FROM addresses ORDER BY address")?;
        let rows = stmt.query_map([], |row| row.get::<_, i64>(0))?;
        rows.map(|r| Ok(LoRaAddr::from_raw(r? as u16)))
            .collect::<Result<Vec<_>, _>>()
    }

    // ── Block writes ──────────────────────────────────────────────────────────

    /// Insert or replace a block. The address must already exist.
    pub fn write_block(
        &self,
        address: LoRaAddr,
        block_id: u64,
        data: &[u8],
    ) -> Result<(), rusqlite::Error> {
        self.conn.execute(
            "INSERT OR REPLACE INTO blocks (block_id, address, data)
             VALUES (?1, ?2, ?3)",
            params![block_id as i64, address.to_raw() as i64, data],
        )?;
        Ok(())
    }

    /// Delete a single block.
    pub fn delete_block(&self, address: LoRaAddr, block_id: u64) -> Result<(), rusqlite::Error> {
        self.conn.execute(
            "DELETE FROM blocks WHERE block_id = ?1 AND address = ?2",
            params![block_id as i64, address.to_raw() as i64],
        )?;
        Ok(())
    }

    // ── Block reads ───────────────────────────────────────────────────────────

    /// Read a single block. Returns `None` if absent.
    pub fn read_block(
        &self,
        address: LoRaAddr,
        block_id: u64,
    ) -> Result<Option<Vec<u8>>, rusqlite::Error> {
        let result = self
            .conn
            .query_row(
                "SELECT data FROM blocks WHERE block_id = ?1 AND address = ?2",
                params![block_id as i64, address.to_raw() as i64],
                |row| row.get::<_, Vec<u8>>(0),
            )
            .optional()?;
        Ok(result)
    }

    /// Returns the first block_id strictly after `after` that is absent for `address`.
    /// Returns `None` if all blocks from `after+1` to `BLOCK_ID_MAX` are present,
    /// which in practice won't happen.
    pub fn first_missing_from(
        &self,
        address: LoRaAddr,
        after: u64,
    ) -> Result<u64, rusqlite::Error> {
        // Walk the present block_ids starting from `start` in order, and find
        // the first gap. We stream rather than pulling everything into memory.
        let mut stmt = self.conn.prepare(
            "SELECT block_id FROM blocks
             WHERE address = ?1 AND block_id >= ?2
             ORDER BY block_id",
        )?;

        let mut expected = after;
        let rows = stmt.query_map(params![address.to_raw() as i64, after as i64], |row| {
            row.get::<_, i64>(0)
        })?;

        for row in rows {
            let id = row? as u64;
            if id != expected {
                // There's a gap: expected..id, so `expected` is the first missing one.
                return Ok(expected);
            }
            expected += 1;
        }

        // Ran out of rows — `expected` is the first missing id.
        Ok(expected)
    }

    /// Check whether every block_id in `first..=last` is present for the given address.
    pub fn all_present(
        &self,
        address: LoRaAddr,
        first: u64,
        last: u64,
    ) -> Result<bool, rusqlite::Error> {
        let expected = (last - first + 1) as i64;
        let count: i64 = self.conn.query_row(
            "SELECT COUNT(*) FROM blocks
             WHERE address = ?1 AND block_id BETWEEN ?2 AND ?3",
            params![address.to_raw() as i64, first as i64, last as i64],
            |row| row.get(0),
        )?;
        Ok(count == expected)
    }

    /// Return all block_ids present for `address` in `first..=last`, in order.
    pub fn present_in_range(
        &self,
        address: LoRaAddr,
        first: u64,
        last: u64,
    ) -> Result<Vec<u64>, rusqlite::Error> {
        let mut stmt = self.conn.prepare(
            "SELECT block_id FROM blocks
             WHERE address = ?1 AND block_id BETWEEN ?2 AND ?3
             ORDER BY block_id",
        )?;
        let rows = stmt.query_map(
            params![address.to_raw() as i64, first as i64, last as i64],
            |row| row.get::<_, i64>(0),
        )?;
        rows.map(|r| Ok(r? as u64)).collect::<Result<Vec<_>, _>>()
    }

    /// Return contiguous runs of present block_ids in `first..=last`.
    /// E.g. if 1,2,3,7,8 are present → [(1,3), (7,8)].
    pub fn present_ranges(
        &self,
        address: LoRaAddr,
        first: u64,
        last: u64,
    ) -> Result<Vec<(u64, u64)>, rusqlite::Error> {
        let ids = self.present_in_range(address, first, last)?;
        Ok(contiguous_runs(&ids))
    }

    /// Return block_ids in `first..=last` that are *missing*.
    pub fn missing_in_range(
        &self,
        address: LoRaAddr,
        first: u64,
        last: u64,
    ) -> Result<Vec<u64>, rusqlite::Error> {
        let present = self.present_in_range(address, first, last)?;
        let present_set: std::collections::HashSet<u64> = present.into_iter().collect();
        Ok((first..=last)
            .filter(|id| !present_set.contains(id))
            .collect())
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn db_path() -> PathBuf {
    let exe = std::env::current_exe().expect("cannot resolve executable path");
    // Resolve symlinks so we get the real location (common on Linux).
    let exe = std::fs::canonicalize(&exe).unwrap_or(exe);
    exe.parent()
        .expect("executable has no parent directory")
        .join("database.sqlite")
}

/// Collapse a sorted list of u64s into contiguous runs (inclusive ranges).
pub fn contiguous_runs(ids: &[u64]) -> Vec<(u64, u64)> {
    let mut runs = Vec::new();
    let mut iter = ids.iter().copied();
    let Some(mut run_start) = iter.next() else {
        return runs;
    };
    let mut run_end = run_start;

    for id in iter {
        if id == run_end + 1 {
            run_end = id;
        } else {
            runs.push((run_start, run_end));
            run_start = id;
            run_end = id;
        }
    }
    runs.push((run_start, run_end));
    runs
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn open_mem() -> Database {
        Database::open_at(":memory:").unwrap()
    }

    #[test]
    fn roundtrip_block() {
        let db = open_mem();
        db.add_address(LoRaAddr::from_raw(0x0001)).unwrap();
        db.write_block(LoRaAddr::from_raw(0x0001), 42, &[1, 2, 3, 4])
            .unwrap();
        let data = db
            .read_block(LoRaAddr::from_raw(0x0001), 42)
            .unwrap()
            .unwrap();
        assert_eq!(data, vec![1, 2, 3, 4]);
    }

    #[test]
    fn all_present_true() {
        let db = open_mem();
        db.add_address(LoRaAddr::from_raw(0x0010)).unwrap();
        for id in 5u64..=9 {
            db.write_block(LoRaAddr::from_raw(0x0010), id, &[0])
                .unwrap();
        }
        assert!(db.all_present(LoRaAddr::from_raw(0x0010), 5, 9).unwrap());
    }

    #[test]
    fn all_present_false() {
        let db = open_mem();
        db.add_address(LoRaAddr::from_raw(0x0010)).unwrap();
        for id in [5u64, 6, 8, 9] {
            db.write_block(LoRaAddr::from_raw(0x0010), id, &[0])
                .unwrap();
        }
        assert!(!db.all_present(LoRaAddr::from_raw(0x0010), 5, 9).unwrap());
    }

    #[test]
    fn present_ranges_basic() {
        let addr = LoRaAddr::from_raw(0x0001);
        let db = open_mem();
        db.add_address(addr).unwrap();
        for id in [1u64, 2, 3, 7, 8] {
            db.write_block(addr, id, &[0]).unwrap();
        }
        let ranges = db.present_ranges(addr, 1, 10).unwrap();
        assert_eq!(ranges, vec![(1, 3), (7, 8)]);
    }

    #[test]
    fn first_missing_after_basic() {
        let db = open_mem();
        db.add_address(LoRaAddr::from_raw(0x0001)).unwrap();
        for id in [10u64, 11, 12, 15, 16] {
            db.write_block(LoRaAddr::from_raw(0x0001), id, &[0])
                .unwrap();
        }
        // Gap at 13 after a run of 10,11,12
        assert_eq!(
            db.first_missing_from(LoRaAddr::from_raw(0x0001), 10)
                .unwrap(),
            13
        );
        // After 12, same gap
        assert_eq!(
            db.first_missing_from(LoRaAddr::from_raw(0x0001), 12)
                .unwrap(),
            13
        );
        // 13 itself is missing, so after 12 → 13; after 13 → 14
        assert_eq!(
            db.first_missing_from(LoRaAddr::from_raw(0x0001), 13)
                .unwrap(),
            13
        );
        // After the last present block, next sequential id is missing
        assert_eq!(
            db.first_missing_from(LoRaAddr::from_raw(0x0001), 16)
                .unwrap(),
            17
        );
    }
}
