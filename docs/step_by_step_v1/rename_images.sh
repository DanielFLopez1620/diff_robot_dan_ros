#!/bin/bash

count=0
for file in $(ls *.jpg | sort); do
    new_name=$(printf "L%02d.jpg" "$count")
    echo "Renaming: $file -> $new_name"
    mv "$file" "$new_name"
    ((count++))
done
