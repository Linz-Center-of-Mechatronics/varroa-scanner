#!/bin/bash

mkdir _static
sphinx-apidoc -o . ../beeclient
make clean html