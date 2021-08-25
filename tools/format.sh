#!/bin/bash
cd "$(dirname "$0")"
cd ..

# Add the license file.
files=$(find . -name "*.h" -o -name "*.c")

# Use clang-format.
for file in $files
do
  clang-format -i -style=GNU $file
done
