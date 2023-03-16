Code from https://github.com/opencv/opencv/blob/4.x/doc/pattern_tools/gen_pattern.py

```
python3 gen_pattern.py -o radon_checkerboard.svg --rows 8 --columns 11 --type radon_checkerboard -s 25 -m 9 6 -w 297 -h 210

python3 gen_pattern.py -o radon_checkerboard.svg --rows 6 --columns 9 --type radon_checkerboard -s 31 -m 8 5 -w 297 -h 210

inkscape --export-filename=radon_checkerboard.pdf radon_check
erboard.svg
```
