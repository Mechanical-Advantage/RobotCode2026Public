// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.wpilogxz;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import org.tukaani.xz.LZMA2Options;
import org.tukaani.xz.XZOutputStream;

public class WPILOGXZEncoder {
  private final XZOutputStream outputStream;
  private int nextEntryId = 1;

  // Reusable buffers
  private final ByteArrayOutputStream payloadBuffer = new ByteArrayOutputStream(1024);
  private final ByteBuffer primitiveBuffer = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN);

  // Buffer for assembling the record header (Bitfield + ID + Size + Timestamp)
  // Max size: 1 (Bitfield) + 4 (ID) + 4 (Size) + 8 (TS) = 17 bytes
  private final byte[] headerBuffer = new byte[17];

  public WPILOGXZEncoder(OutputStream out) throws IOException {
    // LZMA2 Level 6 (8MB Dict)
    this.outputStream = new XZOutputStream(out, new LZMA2Options(6));
  }

  public void close() throws IOException {
    outputStream.close();
  }

  public void flush() throws IOException {
    outputStream.flush();
  }

  public void writeHeader(String extraHeader) throws IOException {
    outputStream.write("WPILOG".getBytes(StandardCharsets.UTF_8));
    outputStream.write(0x00);
    outputStream.write(0x01);

    byte[] extraBytes = extraHeader.getBytes(StandardCharsets.UTF_8);
    // Write length manually to avoid VarInt logic here (Standard integer)
    ByteBuffer lenBuf = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
    lenBuf.putInt(extraBytes.length);
    outputStream.write(lenBuf.array());
    outputStream.write(extraBytes);
  }

  public int startEntry(String name, String type, String metadata, long timestamp)
      throws IOException {
    int id = nextEntryId++;
    payloadBuffer.reset();
    payloadBuffer.write(0);
    writeIntToBuffer(id, 4);
    writeStringToBuffer(name);
    writeStringToBuffer(type);
    writeStringToBuffer(metadata);
    writeRecord(0, timestamp, payloadBuffer.toByteArray());
    return id;
  }

  public void setMetadata(int id, String metadata, long timestamp) throws IOException {
    payloadBuffer.reset();
    payloadBuffer.write(2);
    writeIntToBuffer(id, 4);
    writeStringToBuffer(metadata);
    writeRecord(0, timestamp, payloadBuffer.toByteArray());
  }

  // --- Data Appenders ---

  public void appendRaw(int id, byte[] value, long timestamp) throws IOException {
    writeRecord(id, timestamp, value);
  }

  public void appendBoolean(int id, boolean value, long timestamp) throws IOException {
    writeRecord(id, timestamp, new byte[] {(byte) (value ? 1 : 0)});
  }

  public void appendInteger(int id, long value, long timestamp) throws IOException {
    primitiveBuffer.clear();
    primitiveBuffer.putLong(value);
    writeRecord(id, timestamp, primitiveBuffer.array());
  }

  public void appendFloat(int id, float value, long timestamp) throws IOException {
    primitiveBuffer.clear();
    primitiveBuffer.putFloat(value);
    writeRecord(id, timestamp, primitiveBuffer.array(), 4);
  }

  public void appendDouble(int id, double value, long timestamp) throws IOException {
    primitiveBuffer.clear();
    primitiveBuffer.putDouble(value);
    writeRecord(id, timestamp, primitiveBuffer.array());
  }

  public void appendString(int id, String value, long timestamp) throws IOException {
    writeRecord(id, timestamp, value.getBytes(StandardCharsets.UTF_8));
  }

  public void appendBooleanArray(int id, boolean[] value, long timestamp) throws IOException {
    byte[] bytes = new byte[value.length];
    for (int i = 0; i < value.length; i++) bytes[i] = (byte) (value[i] ? 1 : 0);
    writeRecord(id, timestamp, bytes);
  }

  public void appendIntegerArray(int id, long[] value, long timestamp) throws IOException {
    ByteBuffer bb = ByteBuffer.allocate(value.length * 8).order(ByteOrder.LITTLE_ENDIAN);
    for (long v : value) bb.putLong(v);
    writeRecord(id, timestamp, bb.array());
  }

  public void appendFloatArray(int id, float[] value, long timestamp) throws IOException {
    ByteBuffer bb = ByteBuffer.allocate(value.length * 4).order(ByteOrder.LITTLE_ENDIAN);
    for (float v : value) bb.putFloat(v);
    writeRecord(id, timestamp, bb.array());
  }

  public void appendDoubleArray(int id, double[] value, long timestamp) throws IOException {
    ByteBuffer bb = ByteBuffer.allocate(value.length * 8).order(ByteOrder.LITTLE_ENDIAN);
    for (double v : value) bb.putDouble(v);
    writeRecord(id, timestamp, bb.array());
  }

  public void appendStringArray(int id, String[] value, long timestamp) throws IOException {
    ByteArrayOutputStream bos = new ByteArrayOutputStream();
    ByteBuffer lenBuf = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
    lenBuf.putInt(value.length);
    bos.write(lenBuf.array());
    for (String s : value) {
      byte[] sBytes = s.getBytes(StandardCharsets.UTF_8);
      lenBuf.clear();
      lenBuf.putInt(sBytes.length);
      bos.write(lenBuf.array());
      bos.write(sBytes);
    }
    writeRecord(id, timestamp, bos.toByteArray());
  }

  // --- Internal Helpers ---

  private void writeRecord(int entryId, long timestamp, byte[] payload) throws IOException {
    writeRecord(entryId, timestamp, payload, payload.length);
  }

  /**
   * Optimizes writing by assembling the header in a byte array to strictly minimize calls to
   * outputStream.write().
   */
  private void writeRecord(int entryId, long timestamp, byte[] payload, int length)
      throws IOException {
    int idLen = bytesNeeded(entryId);
    int sizeLen = bytesNeeded(length);
    int tsLen = bytesNeeded(timestamp);

    // 1. Calculate Bitfield
    int bitfield = (idLen - 1) | ((sizeLen - 1) << 2) | ((tsLen - 1) << 4);

    // 2. Assemble Header in 'headerBuffer'
    int pos = 0;
    headerBuffer[pos++] = (byte) bitfield;

    // Write ID (Little Endian VarInt)
    long tempVal = entryId;
    for (int i = 0; i < idLen; i++) {
      headerBuffer[pos++] = (byte) (tempVal & 0xFF);
      tempVal >>>= 8;
    }

    // Write Size
    tempVal = length;
    for (int i = 0; i < sizeLen; i++) {
      headerBuffer[pos++] = (byte) (tempVal & 0xFF);
      tempVal >>>= 8;
    }

    // Write Timestamp
    tempVal = timestamp;
    for (int i = 0; i < tsLen; i++) {
      headerBuffer[pos++] = (byte) (tempVal & 0xFF);
      tempVal >>>= 8;
    }

    // 3. Write Header (One Call)
    outputStream.write(headerBuffer, 0, pos);

    // 4. Write Payload (One Call)
    outputStream.write(payload, 0, length);
  }

  private int bytesNeeded(long val) {
    if (val == 0) return 1;
    if (val < 0) return 8;
    if (val < (1L << 8)) return 1;
    if (val < (1L << 16)) return 2;
    if (val < (1L << 24)) return 3;
    if (val < (1L << 32)) return 4;
    if (val < (1L << 40)) return 5;
    if (val < (1L << 48)) return 6;
    if (val < (1L << 56)) return 7;
    return 8;
  }

  // Helper for internal payload buffer (uncompressed construction)
  private void writeIntToBuffer(int val, int len) {
    for (int i = 0; i < len; i++) {
      payloadBuffer.write((val >> (i * 8)) & 0xFF);
    }
  }

  private void writeStringToBuffer(String str) throws IOException {
    byte[] b = str.getBytes(StandardCharsets.UTF_8);
    writeIntToBuffer(b.length, 4);
    payloadBuffer.write(b);
  }
}
