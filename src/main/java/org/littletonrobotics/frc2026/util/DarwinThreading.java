// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.Structure;

public class DarwinThreading {
  private static final int THREAD_TIME_CONSTRAINT_POLICY = 2;
  private static final int KERN_SUCCESS = 0;

  public interface MachLib extends Library {
    MachLib INSTANCE = Native.load("System", MachLib.class);

    // Get the mach thread port for the current thread
    int mach_thread_self();

    // Get hardware timebase info (to convert ns to ticks)
    int mach_timebase_info(MachTimebaseInfo info);

    // Set the thread policy
    int thread_policy_set(int thread, int flavor, Structure policyInfo, int count);
  }

  @Structure.FieldOrder({"numer", "denom"})
  public static class MachTimebaseInfo extends Structure {
    public int numer;
    public int denom;
  }

  @Structure.FieldOrder({"period", "computation", "constraint", "preemptible"})
  public static class ThreadTimeConstraintPolicy extends Structure {
    public int period;
    public int computation;
    public int constraint;
    public int preemptible; // boolean_t (0=false, 1=true)
  }

  public static void setTimeConstraintPolicy(int periodMs, int computationMs, int constraintMs) {
    MachLib lib = MachLib.INSTANCE;

    // 1. Get timebase info (hardware ticks conversion)
    MachTimebaseInfo timebase = new MachTimebaseInfo();
    if (lib.mach_timebase_info(timebase) != KERN_SUCCESS) {
      throw new RuntimeException("Failed to get mach_timebase_info");
    }

    // Helper to convert MS to mach absolute time
    // Formula: (nanos * denom) / numer
    long msToMach = (1_000_000L * timebase.denom) / timebase.numer;

    // 2. Set up policy
    ThreadTimeConstraintPolicy policy = new ThreadTimeConstraintPolicy();
    policy.period = (int) (periodMs * msToMach);
    policy.computation = (int) (computationMs * msToMach);
    policy.constraint = (int) (constraintMs * msToMach);
    policy.preemptible = 0;

    // 3. Apply to current thread
    int threadPort = lib.mach_thread_self();
    // Count is size in 'natural_t' (4-byte ints)
    int policyCount = policy.size() / 4;

    int result =
        lib.thread_policy_set(threadPort, THREAD_TIME_CONSTRAINT_POLICY, policy, policyCount);

    if (result != KERN_SUCCESS) {
      if (result == 4) { // KERN_INVALID_ARGUMENT
        throw new RuntimeException(
            "Failed to configure time constraint policy. Check permissions.");
      }
      throw new RuntimeException("Mach call failed with kernel error code: " + result);
    }
  }
}
