#!/bin/bash
num_changes=`$1 -style=file -output-replacements-xml "${@:2}" | grep -c "<replacement "`

if [ "$num_changes" = "0" ]; then
  echo "Every file seems to comply with our code convention."
  exit 0
else
  echo "Found" $num_changes "necessary changes."
  exit 0
fi
