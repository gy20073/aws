#!/usr/env bash

PYSTYLE=$(which pycodestyle)
if [ $? -ne 0 ]; then
    pip install pycodestyle
fi

CPPSTYLE=$(which cpplint)
if [ $? -ne 0 ]; then
    pip install cpplint
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PC_PATH=$DIR/../.git/hooks/pre-commit
if [ -f $PC_PATH ]; then
   rm $PC_PATH
fi
ln -s $DIR/pre-commit.sh $DIR/../.git/hooks/pre-commit
chmod +x $DIR/pre-commit.sh
