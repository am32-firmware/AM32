#!/bin/sh

find ./Src/ -regex '.*\.\(h\|c\)' -not -path "*/Drivers/*" -exec clang-format -style=WebKit -i {} \;
find ./Inc/ -regex '.*\.\(h\|c\)' -not -path "*/Drivers/*" -exec clang-format -style=WebKit -i {} \;
find ./Mcu/ -regex '.*\.\(h\|c\)' -not -path "*/Drivers/*" -exec clang-format -style=WebKit -i {} \;

echo "Formatting done"
