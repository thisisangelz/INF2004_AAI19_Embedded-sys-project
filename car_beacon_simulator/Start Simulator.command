#!/bin/zsh
cd -- "${0:A:h}"
echo 'Open http://127.0.0.1:8765 in your browser. Leave this window open while testing.'
python3 simulator.py
