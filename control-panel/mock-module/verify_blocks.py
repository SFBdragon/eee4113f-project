#!/bin/python

import json
import os
from sys import argv


def verify_binary_data(json_path, bin_path):
    # Load the mapping from JSON
    try:
        with open(json_path, "r") as f:
            mapping = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error loading JSON: {e}")
        return

    # Create the reference byte sequence (0 to 200)
    # bytes() handles the conversion of a list of ints to actual bytes
    reference_data = bytes(range(200))

    if not os.path.exists(bin_path):
        print(f"Error: {bin_path} not found.")
        return

    any_blocks_failed = False
    block_ids = set()

    with open(bin_path, "rb") as bin_file:
        for block in mapping:
            b_id = block.get("block_id")
            offset = block.get("offset")
            length = block.get("length")

            bin_file.seek(offset)
            chunk = bin_file.read(length)

            expected = reference_data[:length]

            if chunk == expected:
                print(f"Block {b_id:02}: MATCH (Offset: {offset}, Length: {length})")
            else:
                print(chunk)
                print()
                print(expected)
                print()
                print(f"Block {b_id:02}: FAILED verification.")
                any_blocks_failed = True

            block_ids.add(b_id)

    if block_ids == set(range(100)):
        print("All blocks accounted for.")
    else:
        print(f"Blocks are missing. Number of unique blocks = {len(block_ids)}")

    if any_blocks_failed:
        print("Some blocks were not correct!")
    else:
        print("All blocks matched.")


if __name__ == "__main__":
    path = argv[1]
    json_path = os.path.join(path, "data.json")
    bin_path = os.path.join(path, "data.bin")
    print(f"Reading from {json_path}")
    verify_binary_data(json_path, bin_path)
