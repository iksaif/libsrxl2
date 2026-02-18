#!/usr/bin/env sh

# install marker if needed
if ! command -v marker &> /dev/null
then
    pip install marker-pdf
fi

find .. -name "*.pdf" -exec cp {} . \;

marker .

rm *.pdf
