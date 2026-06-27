// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.gradlerio;

import edu.wpi.first.deployutils.deploy.cache.AbstractCacheMethod;
import edu.wpi.first.deployutils.deploy.context.DeployContext;
import edu.wpi.first.deployutils.log.ETLogger;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import javax.inject.Inject;
import org.codehaus.groovy.runtime.EncodingGroovyMethods;
import org.gradle.api.logging.Logger;
import org.gradle.api.logging.Logging;

public class MacMiniMd5SumCacheMethod extends AbstractCacheMethod {
  private Logger log = Logging.getLogger(MacMiniMd5SumCacheMethod.class);
  private int csI = 0;

  @Inject
  public MacMiniMd5SumCacheMethod(String name) {
    super(name);
  }

  @Override
  public boolean compatible(DeployContext context) {
    ETLogger logger = context.getLogger();
    if (logger != null) {
      logger.silent(true);
    }
    String sum = context.execute("echo test | md5sum 2> /dev/null").getResult();
    if (logger != null) {
      logger.silent(false);
    }

    return !sum.isEmpty() && sum.split(" ")[0].equalsIgnoreCase("d8e8fca2dc0f896fd7cb4cb0031ba249");
  }

  String localChecksumsText(Map<String, File> files) {
    MessageDigest md;
    try {
      md = MessageDigest.getInstance("MD5");
    } catch (NoSuchAlgorithmException e1) {
      throw new RuntimeException(e1);
    }
    Optional<String> sums =
        files.entrySet().stream()
            .map(
                entry -> {
                  md.reset();
                  try {
                    md.update(Files.readAllBytes(entry.getValue().toPath()));
                  } catch (IOException e) {
                    throw new RuntimeException(e);
                  }
                  String local = EncodingGroovyMethods.encodeHex(md.digest()).toString();
                  return local + " *" + entry.getKey();
                })
            .reduce((a, b) -> a + "\n" + b);
    if (sums.isEmpty()) {
      return null;
    }
    return sums.get();
  }

  @Override
  public Set<String> needsUpdate(DeployContext context, Map<String, File> files) {
    ETLogger logger = context.getLogger();
    if (logger != null) {
      logger.silent(true);
    }

    int cs = csI++;

    log.debug("Comparing Checksums " + cs + "...");
    String localChecksums = localChecksumsText(files);

    if (log.isDebugEnabled()) {
      log.debug("Local Checksums " + cs + ":");
      log.debug(localChecksums);
    }

    String result =
        context
            .execute(
                "echo '"
                    + localChecksums
                    + "' | md5sum -c - 2> /dev/null") // Modified to work on macOS
            .getResult();

    if (log.isDebugEnabled()) {
      log.debug("Remote Checksums " + cs + ":");
      log.debug(result);
    }

    List<String> upToDate =
        Arrays.stream(result.split("\n"))
            .map(x -> x.split(":"))
            .filter(ls -> ls[ls.length - 1].trim().equalsIgnoreCase("ok"))
            .map(ls -> ls[0])
            .collect(Collectors.toList());

    if (logger != null) {
      logger.silent(false);
    }

    return files.keySet().stream()
        .filter(name -> !upToDate.contains(name))
        .collect(Collectors.toSet());
  }
}
