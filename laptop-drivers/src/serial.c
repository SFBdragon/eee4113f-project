// serial.c
//
// POSIX termios-based serial port. Works on Mac and Linux.
// No external dependencies.

// Feature test macros: required on Linux to get CRTSCTS, B115200, clock_gettime.
// On macOS, _DARWIN_C_SOURCE serves the same role.
// Important: do NOT define _POSIX_C_SOURCE alone on macOS — it hides BSD extensions.
#if defined(__linux__)
  #ifndef _DEFAULT_SOURCE
  #define _DEFAULT_SOURCE
  #endif
  #ifndef _POSIX_C_SOURCE
  #define _POSIX_C_SOURCE 200809L
  #endif
#elif defined(__APPLE__)
  #ifndef _DARWIN_C_SOURCE
  #define _DARWIN_C_SOURCE
  #endif
#endif

#include "../include/serial.h"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#ifdef __APPLE__
  // Apple's <termios.h> only exposes baud constants up to B230400.
  // For 460800, 921600, and arbitrary rates, the kernel provides
  // IOSSIOSPEED via <IOKit/serial/ioss.h>. We set a placeholder baud
  // with tcsetattr, then override it via ioctl.
  #include <IOKit/serial/ioss.h>
  #include <sys/ioctl.h>
#endif

// Map a baud-rate int to the speed_t constant termios wants.
// Add more cases as needed.
static speed_t baud_to_speed(uint32_t baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default:     return B0;
    }
}

serial_handle_t serial_open(const char *device, uint32_t baud) {
    speed_t speed = baud_to_speed(baud);

#ifdef __APPLE__
    // On macOS, fall back to IOSSIOSPEED for baud rates not in <termios.h>
    // (typically anything above 230400). We still need a valid speed_t to
    // pass through cfsetispeed/cfsetospeed before the ioctl override.
    bool needs_iossiospeed = false;
    if (speed == B0) {
        speed = B9600;          // placeholder; will be overridden below
        needs_iossiospeed = true;
    }
#else
    if (speed == B0) {
        fprintf(stderr, "serial_open: unsupported baud %u\n", baud);
        return SERIAL_INVALID;
    }
#endif

    // O_NOCTTY: don't make this our controlling terminal.
    // O_NONBLOCK: open without blocking; we'll switch to blocking reads via select later.
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "serial_open: open(%s): %s\n", device, strerror(errno));
        return SERIAL_INVALID;
    }

    // Clear O_NONBLOCK for normal reads — we'll use select() for timeouts.
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "serial_open: tcgetattr: %s\n", strerror(errno));
        close(fd);
        return SERIAL_INVALID;
    }

    // Set baud both directions.
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1, no flow control.
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8 data bits
    tty.c_cflag &= ~PARENB;                          // no parity
    tty.c_cflag &= ~CSTOPB;                          // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                         // no hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;                   // enable receiver, ignore modem ctrl lines

    // Raw input: no canonical mode, no echo, no signal generation.
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);

    // Raw input flags: no software flow control, no special-character handling.
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Raw output: don't translate \n to \r\n or similar.
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // VMIN=0, VTIME=0: read() returns immediately with whatever bytes are available.
    // We rely on select() for timeouts.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "serial_open: tcsetattr: %s\n", strerror(errno));
        close(fd);
        return SERIAL_INVALID;
    }

#ifdef __APPLE__
    if (needs_iossiospeed) {
        // IOSSIOSPEED takes a speed_t (unsigned long on macOS) by pointer
        // containing the actual baud rate in bps. Must be called AFTER
        // tcsetattr; if you call it before, tcsetattr will reset the speed.
        speed_t bps = (speed_t)baud;
        if (ioctl(fd, IOSSIOSPEED, &bps) == -1) {
            fprintf(stderr, "serial_open: IOSSIOSPEED(%u): %s\n",
                    baud, strerror(errno));
            close(fd);
            return SERIAL_INVALID;
        }
    }
#endif

    // Drain any junk that was buffered before we configured the port.
    tcflush(fd, TCIOFLUSH);

    return fd;
}

void serial_close(serial_handle_t h) {
    if (h != SERIAL_INVALID) {
        close(h);
    }
}

int serial_write_all(serial_handle_t h, const uint8_t *data, size_t len) {
    if (h == SERIAL_INVALID) return -1;
    size_t written = 0;
    while (written < len) {
        ssize_t n = write(h, data + written, len - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        written += (size_t)n;
    }
    return (int)written;
}

int serial_read_byte(serial_handle_t h, uint8_t *out, uint32_t timeout_ms) {
    if (h == SERIAL_INVALID) return -1;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(h, &rfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int sel = select(h + 1, &rfds, NULL, NULL, &tv);
    if (sel < 0) {
        if (errno == EINTR) return 0;  // treat as timeout for simplicity
        return -1;
    }
    if (sel == 0) return 0;  // timeout

    ssize_t n = read(h, out, 1);
    if (n == 1) return 1;
    if (n == 0) return 0;    // EOF or no data; treat as timeout
    return -1;
}

void serial_flush_input(serial_handle_t h) {
    if (h == SERIAL_INVALID) return;
    tcflush(h, TCIFLUSH);
}
