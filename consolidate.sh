#!/bin/bash

target_dir=$1

if ! test -d "$target_dir/0"; then
	echo "Cannot consolidate: output was not generated with symmetry mode"
	exit
fi
if ! test -d "$target_dir/1"; then
	echo "Cannot consolidate: output was not generated with symmetry mode"
	exit
fi

num_files_0=$(find "$target_dir/0" -maxdepth 1 -type f -name "*.stl" | wc -l)
num_files_1=$(find "$target_dir/1" -maxdepth 1 -type f -name "*.stl" | wc -l)

find "$target_dir/0" -maxdepth 1 -type f -iname "*.stl" | while read -r file; do
	fullname=$(basename "$file")
	name="${fullname%.*}"
	mv "$file" "$target_dir/$fullname"
done
rm -r "$target_dir/0"

find "$target_dir/1" -maxdepth 1 -type f -iname "*.stl" | while read -r file; do
	fullname=$(basename "$file")
	name="${fullname%.*}"
	mv "$file" "$target_dir/$(($(expr $name) + num_files_0)).stl"
done
rm -r "$target_dir/1"

total_files=$(find "$target_dir" -maxdepth 1 -type f -name "*.stl" | wc -l)
echo "Consolidated $num_files_0 models in $target_dir/0 and $num_files_1 models in /1 into $target_dir/$total_files models in $target_dir."
