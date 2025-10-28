#!/bin/bash


if [ -d Debug ]
then
	make -C Debug  all 
fi 

if [ -d Release ]
then
        make -C Release all
fi

