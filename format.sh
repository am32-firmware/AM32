#!/bin/sh

find ./Src/ -regex '.*\.\(h\|c\)' -not -path "*/Drivers/*" -exec clang-format -style=file -i {} \;
find ./Inc/ -regex '.*\.\(h\|c\)' -not -path "*/Drivers/*" -exec clang-format -style=file -i {} \;
find ./Mcu/ -regex '.*\.\(h\|c\)' -not -path "*/Drivers/*" -exec clang-format -style=file -i {} \;

echo "Formatting done"
