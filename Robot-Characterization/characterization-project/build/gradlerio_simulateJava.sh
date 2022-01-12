#!/bin/bash

export HALSIM_EXTENSIONS=/Users/derickson/Code/FRC-2022/Robot-Characterization/characterization-project/build/tmp/expandedArchives/halsim_gui-2021.1.2-osxx86-64.zip_cebe157c0370b0beb19e3bf47444ccd5/osx/x86-64/shared/libhalsim_gui.dylib
export LD_LIBRARY_PATH=/Users/derickson/Code/FRC-2022/Robot-Characterization/characterization-project/build/tmp/jniExtractDir
export DYLD_FALLBACK_LIBRARY_PATH=/Users/derickson/Code/FRC-2022/Robot-Characterization/characterization-project/build/tmp/jniExtractDir
export DYLD_LIBRARY_PATH=/Users/derickson/Code/FRC-2022/Robot-Characterization/characterization-project/build/tmp/jniExtractDir
"/Users/derickson/wpilib/2021/jdk/bin/java" -Djava.library.path="/Users/derickson/Code/FRC-2022/Robot-Characterization/characterization-project/build/tmp/jniExtractDir" -XstartOnFirstThread -jar "/Users/derickson/Code/FRC-2022/Robot-Characterization/characterization-project/build/libs/characterization-project.jar"
