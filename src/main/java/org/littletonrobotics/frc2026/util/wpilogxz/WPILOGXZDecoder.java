// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.wpilogxz;

import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.ByteArrayOutputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.function.Consumer;
import org.tukaani.xz.XZInputStream;

/** Data log reader for XZ encoded logs (reads logs written by the DataLog class). */
public class WPILOGXZDecoder implements Iterable<DataLogRecord> {
  private final ByteBuffer buffer;
  private static final Constructor<DataLogRecord> recordConstructor;

  static {
    try {
      recordConstructor =
          DataLogRecord.class.getDeclaredConstructor(int.class, long.class, ByteBuffer.class);
      recordConstructor.setAccessible(true);
    } catch (NoSuchMethodException e) {
      throw new RuntimeException("Failed to access DataLogRecord constructor", e);
    }
  }

  /**
   * Constructs from a byte buffer.
   *
   * @param buffer byte buffer
   */
  public WPILOGXZDecoder(ByteBuffer buffer) {
    this.buffer = buffer;
    this.buffer.order(ByteOrder.LITTLE_ENDIAN);
  }

  /**
   * Constructs from a file. The file is expected to be XZ encoded.
   *
   * @param filename filename (e.g. "log.wpilogxz")
   * @throws IOException if unable to open/read file
   */
  public WPILOGXZDecoder(String filename) throws IOException {
    try (FileInputStream fis = new FileInputStream(filename);
        XZInputStream in = new XZInputStream(fis)) {
      // Read all bytes from the compressed stream into memory
      ByteArrayOutputStream out = new ByteArrayOutputStream();
      byte[] buffer = new byte[8192];
      int read;
      try {
        while ((read = in.read(buffer)) != -1) {
          out.write(buffer, 0, read);
        }
      } catch (IOException e) {
        // Ignore errors if we have read some data (likely a truncated file)
        if (out.size() == 0) {
          throw e;
        }
      }
      this.buffer = ByteBuffer.wrap(out.toByteArray());
      this.buffer.order(ByteOrder.LITTLE_ENDIAN);
    }
  }

  /**
   * Returns true if the data log is valid (e.g. has a valid header).
   *
   * @return True if valid, false otherwise
   */
  public boolean isValid() {
    return buffer.remaining() >= 12
        && buffer.get(0) == 'W'
        && buffer.get(1) == 'P'
        && buffer.get(2) == 'I'
        && buffer.get(3) == 'L'
        && buffer.get(4) == 'O'
        && buffer.get(5) == 'G'
        && buffer.getShort(6) >= 0x0100;
  }

  /**
   * Gets the data log version. Returns 0 if data log is invalid.
   *
   * @return Version number; most significant byte is major, least significant is minor (so version
   *     1.0 will be 0x0100)
   */
  public short getVersion() {
    if (buffer.remaining() < 12) {
      return 0;
    }
    return buffer.getShort(6);
  }

  /**
   * Gets the extra header data.
   *
   * @return Extra header data
   */
  public String getExtraHeader() {
    ByteBuffer buf = buffer.duplicate();
    buf.order(ByteOrder.LITTLE_ENDIAN);
    buf.position(8);
    int size = buf.getInt();
    byte[] arr = new byte[size];
    buf.get(arr);
    return new String(arr, StandardCharsets.UTF_8);
  }

  @Override
  public void forEach(Consumer<? super DataLogRecord> action) {
    int size = buffer.remaining();
    for (int pos = 12 + buffer.getInt(8); pos < size; pos = getNextRecord(pos)) {
      DataLogRecord record;
      try {
        record = getRecord(pos);
      } catch (NoSuchElementException ex) {
        break;
      }
      action.accept(record);
    }
  }

  @Override
  public Iterator<DataLogRecord> iterator() {
    return new Iterator<DataLogRecord>() {
      private int pos = 12 + buffer.getInt(8);

      @Override
      public boolean hasNext() {
        return pos < buffer.remaining();
      }

      @Override
      public DataLogRecord next() {
        try {
          DataLogRecord record = getRecord(pos);
          pos = getNextRecord(pos);
          return record;
        } catch (NoSuchElementException ex) {
          throw ex;
        }
      }
    };
  }

  private long readVarInt(int pos, int len) {
    long val = 0;
    for (int i = 0; i < len; i++) {
      val |= ((long) (buffer.get(pos + i) & 0xff)) << (i * 8);
    }
    return val;
  }

  DataLogRecord getRecord(int pos) {
    try {
      int lenbyte = buffer.get(pos) & 0xff;
      int entryLen = (lenbyte & 0x3) + 1;
      int sizeLen = ((lenbyte >> 2) & 0x3) + 1;
      int timestampLen = ((lenbyte >> 4) & 0x7) + 1;
      int headerLen = 1 + entryLen + sizeLen + timestampLen;
      int entry = (int) readVarInt(pos + 1, entryLen);
      int size = (int) readVarInt(pos + 1 + entryLen, sizeLen);
      long timestamp = readVarInt(pos + 1 + entryLen + sizeLen, timestampLen);
      // build a slice of the data contents
      ByteBuffer data = buffer.duplicate();
      data.position(pos + headerLen);
      data.limit(pos + headerLen + size);
      try {
        return recordConstructor.newInstance(entry, timestamp, data.slice());
      } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
        throw new RuntimeException("Failed to instantiate DataLogRecord", e);
      }
    } catch (BufferUnderflowException | IndexOutOfBoundsException ex) {
      throw new NoSuchElementException();
    }
  }

  int getNextRecord(int pos) {
    int lenbyte = buffer.get(pos) & 0xff;
    int entryLen = (lenbyte & 0x3) + 1;
    int sizeLen = ((lenbyte >> 2) & 0x3) + 1;
    int timestampLen = ((lenbyte >> 4) & 0x7) + 1;
    int headerLen = 1 + entryLen + sizeLen + timestampLen;

    int size = 0;
    for (int i = 0; i < sizeLen; i++) {
      size |= (buffer.get(pos + 1 + entryLen + i) & 0xff) << (i * 8);
    }
    return pos + headerLen + size;
  }

  int size() {
    return buffer.remaining();
  }
}
