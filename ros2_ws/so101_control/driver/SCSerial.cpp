/*
 * SCSerial.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2022.3.29
 * 作者:
 */

#include "SCSerial.h"
#include <iostream>

SCSerial::SCSerial() {
  IOTimeOut = 100;
  fd = -1;
  txBufLen = 0;
}

SCSerial::SCSerial(u8 End) : SCS(End) {
  IOTimeOut = 100;
  fd = -1;
  txBufLen = 0;
}

SCSerial::SCSerial(u8 End, u8 Level) : SCS(End, Level) {
  IOTimeOut = 100;
  fd = -1;
  txBufLen = 0;
}

speed_t SCSerial::get_baud_constant(int baudRate) {
  switch (baudRate) {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  case 230400:
    return B230400;
  case 460800:
    return B460800;
  case 500000:
    return B500000;
  case 576000:
    return B576000;
  case 921600:
    return B921600;
  case 1000000:
    return B1000000;
  default:
    return B0;
  }
}

bool SCSerial::begin(int baudRate, const char *serialPort) {
  if (fd != -1) {
    close(fd);
    fd = -1;
  }
  // printf("servo port:%s\n", serialPort);
  if (serialPort == NULL)
    return false;
  fd = open(serialPort, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror("open:");
    return false;
  }

  fcntl(fd, F_SETFL, FNDELAY);
  speed_t CR_BAUDRATE = get_baud_constant(baudRate);
  memset(&curopt, 0, sizeof(curopt));
  if (tcgetattr(fd, &curopt) != 0) {
    perror("tcgetattr:");
    close(fd);
    return false;
  }

  cfsetispeed(&curopt, CR_BAUDRATE);
  cfsetospeed(&curopt, CR_BAUDRATE);

  printf("serial speed %d\n", baudRate);

  curopt.c_cflag &= ~CSIZE;
  curopt.c_cflag |= CS8;
  curopt.c_cflag |= CLOCAL | CREAD;
  curopt.c_cflag &= ~PARENB;
  curopt.c_cflag &= ~CSTOPB;
  curopt.c_cflag &= ~CRTSCTS;
  curopt.c_lflag = 0;
  curopt.c_iflag = 0;
  curopt.c_oflag = 0;

  curopt.c_cc[VMIN] = 6;  // VMIN = 1: Block until at least 1 byte is available
  curopt.c_cc[VTIME] = 0; // VTIME = 0: No inter-character timer

  if (tcsetattr(fd, TCSANOW, &curopt) != 0) {
    perror("tcsetattr:");
    close(fd);
    return false;
  }
  return true;
}

int SCSerial::setBaudRate(int baudRate) {
  if (fd == -1) {
    return -1;
  }
  tcgetattr(fd, &orgopt);
  tcgetattr(fd, &curopt);
  speed_t CR_BAUDRATE = baudRate;
  cfsetispeed(&curopt, CR_BAUDRATE);
  cfsetospeed(&curopt, CR_BAUDRATE);
  return 1;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen) {
  int fs_sel;
  fd_set fs_read;
  int rvLen = 0;

  struct timeval time;

  FD_ZERO(&fs_read);
  FD_SET(fd, &fs_read);

  time.tv_sec = 0;
  time.tv_usec = IOTimeOut * 1000;

  // 使用select实现串口的多路通信
  while (1) {
    try {
      fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
      std::cout << "fs_sel: " << fs_sel << std::endl;
      if (fs_sel) {
        rvLen += read(fd, nDat + rvLen, nLen - rvLen);

        if (rvLen < nLen) {
          continue;
        } else {
          return rvLen;
        }
      } else {
        // printf("serial read fd read return 0\n");
        return rvLen;
      }
    } catch (const std::exception &e) {
      std::cerr << "Exception in readSCS: " << e.what() << std::endl;
      return rvLen;
    }
  }
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen) {
  while (nLen--) {
    txBuf[txBufLen++] = *nDat++;
  }
  return txBufLen;
}

int SCSerial::writeSCS(unsigned char bDat) {
  txBuf[txBufLen++] = bDat;
  return txBufLen;
}

void SCSerial::rFlushSCS() { tcflush(fd, TCIFLUSH); }

void SCSerial::wFlushSCS() {
  if (txBufLen) {
    txBufLen = write(fd, txBuf, txBufLen);
    txBufLen = 0;
  }
}

void SCSerial::end() {
  fd = -1;
  close(fd);
}
