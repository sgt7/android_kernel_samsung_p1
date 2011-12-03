#!/bin/bash

find initramfs/ -name .gitignore > list
while read file; do rm -Rf $file; done < list
rm -Rf list
