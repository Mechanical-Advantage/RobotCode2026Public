// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.idun;

import com.google.auto.service.AutoService;
import com.squareup.javapoet.*;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.Processor;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.*;
import javax.lang.model.type.TypeKind;
import javax.tools.Diagnostic;
import javax.tools.FileObject;
import javax.tools.StandardLocation;

@AutoService(Processor.class)
@SupportedAnnotationTypes({
  "org.littletonrobotics.idun.IdunIO",
  "org.littletonrobotics.idun.IdunConstants"
})
@SupportedSourceVersion(SourceVersion.RELEASE_17)
public class IdunAnnotationProcessor extends AbstractProcessor {
  @Override
  public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
    // Process IdunIO annotations
    Optional<? extends TypeElement> idunIoOptional =
        annotations.stream()
            .filter(
                (te) ->
                    te.getQualifiedName().toString().equals("org.littletonrobotics.idun.IdunIO"))
            .findFirst();
    if (!idunIoOptional.isPresent()) {
      return false;
    }
    TypeElement idunIo = idunIoOptional.get();
    roundEnv
        .getElementsAnnotatedWith(idunIo)
        .forEach(
            sourceClass -> {
              // Find input and output classes
              Element inputsClass = null;
              Element outputsClass = null;
              for (var element : sourceClass.getEnclosedElements()) {
                if (element.getKind() == ElementKind.CLASS
                    && element
                        .getSimpleName()
                        .toString()
                        .equals(sourceClass.getSimpleName().toString() + "Inputs")) {
                  inputsClass = element;
                }
                if (element.getKind() == ElementKind.CLASS
                    && element
                        .getSimpleName()
                        .toString()
                        .equals(sourceClass.getSimpleName().toString() + "Outputs")) {
                  outputsClass = element;
                }
              }
              if (inputsClass == null) {
                processingEnv
                    .getMessager()
                    .printMessage(
                        Diagnostic.Kind.ERROR,
                        "Failed to find inputs class for IO interface",
                        sourceClass);
                return;
              }
              if (outputsClass == null) {
                processingEnv
                    .getMessager()
                    .printMessage(
                        Diagnostic.Kind.ERROR,
                        "Failed to find outputs class for IO interface",
                        sourceClass);
                return;
              }

              // Check for invalid IO types
              for (var ioClass : List.of(inputsClass, outputsClass)) {
                for (var element : ioClass.getEnclosedElements()) {
                  if (element.getKind() == ElementKind.FIELD
                      && !element.asType().toString().equals("double")
                      && !element
                          .asType()
                          .toString()
                          .equals("edu.wpi.first.math.geometry.Rotation2d")
                      && !element.asType().toString().equals("boolean")
                      && !element.asType().toString().equals("byte[]")
                      && !isEnum(element, processingEnv)) {
                    processingEnv
                        .getMessager()
                        .printMessage(
                            Diagnostic.Kind.ERROR,
                            "IO data field is not compatible with Idun",
                            element);
                  }
                }
              }

              // Run IO code generators
              IdunJavaIOGenerator.generate(sourceClass, inputsClass, outputsClass, processingEnv);
              IdunNativeIOGenerator.generate(sourceClass, inputsClass, outputsClass, processingEnv);
            });

    // Process IdunConstants annotations
    Optional<? extends TypeElement> idunConstantsOptional =
        annotations.stream()
            .filter(
                (te) ->
                    te.getQualifiedName()
                        .toString()
                        .equals("org.littletonrobotics.idun.IdunConstants"))
            .findFirst();
    if (!idunConstantsOptional.isPresent()) {
      return false;
    }
    TypeElement idunConstants = idunConstantsOptional.get();
    roundEnv
        .getElementsAnnotatedWith(idunConstants)
        .forEach(
            sourceClass -> {
              IdunNativeConstantsGenerator.generate(sourceClass, processingEnv);
            });

    // Create build constants files
    long buildId = new Random().nextLong();
    TypeSpec.Builder typeBuilder =
        TypeSpec.classBuilder("IdunBuildConstants")
            .addModifiers(Modifier.PUBLIC)
            .addModifiers(Modifier.FINAL)
            .addJavadoc("Constants related to this build of the robot project.");
    typeBuilder.addField(
        FieldSpec.builder(TypeName.LONG, "uid", Modifier.PUBLIC, Modifier.FINAL, Modifier.STATIC)
            .initializer(Long.toString(buildId) + "L")
            .build());
    typeBuilder.addMethod(MethodSpec.constructorBuilder().addModifiers(Modifier.PRIVATE).build());
    JavaFile file = JavaFile.builder("org.littletonrobotics.idun", typeBuilder.build()).build();
    try {
      file.writeTo(processingEnv.getFiler());
    } catch (IOException e) {
      processingEnv
          .getMessager()
          .printMessage(Diagnostic.Kind.ERROR, "Failed to write Idun constants");
      e.printStackTrace();
    }

    try {
      // Only declare buildUID as a symbol
      FileObject declFile =
          processingEnv
              .getFiler()
              .createResource(
                  StandardLocation.NATIVE_HEADER_OUTPUT, "", "idun/IdunBuildConstants.h");
      try (PrintWriter writer = new PrintWriter(declFile.openWriter())) {
        writer.format(
            """
#pragma once

namespace idun {

extern const long long buildUID;

}; // namespace idun
            """);
      }
      // Set value of buildUID
      FileObject valueFile =
          processingEnv
              .getFiler()
              .createResource(
                  StandardLocation.NATIVE_HEADER_OUTPUT, "", "idun/IdunBuildConstants.cpp");
      try (PrintWriter writer = new PrintWriter(valueFile.openWriter())) {
        writer.format(
            """
#include "idun/IdunBuildConstants.h"

namespace idun {

constexpr long long buildUID = %dLL;

}; // namespace idun
            """,
            buildId);
      }
    } catch (IOException e) {
      e.printStackTrace();
    }

    return true;
  }

  static String getPackageName(Element e) {
    while (e != null) {
      if (e.getKind().equals(ElementKind.PACKAGE)) {
        return ((PackageElement) e).getQualifiedName().toString();
      }
      e = e.getEnclosingElement();
    }

    return null;
  }

  static boolean isEnum(Element e, ProcessingEnvironment env) {
    return e.asType().getKind().equals(TypeKind.DECLARED)
        && ((TypeElement) env.getTypeUtils().asElement(e.asType()))
            .getKind()
            .equals(ElementKind.ENUM);
  }
}
