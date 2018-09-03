#!/bin/bash

# add current directory to .bash_aliases

# credit goes to http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was  located
done
CUR_DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

echo "export PATH=$CUR_DIR:\$PATH" >> ~/.bash_aliases

