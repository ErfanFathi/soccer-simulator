#!/bin/sh
protoc -I=../src/proto --python_out=. ../src/proto/commands.proto ../src/proto/observation.proto