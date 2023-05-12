# HOLOLENS CALIBRATION
This folder contains all the useful codes to perform a camera calibration.
For more information refers to [OpenCV Camera Calibration](https://docs.opencv.org/4.7.0/dc/dbb/tutorial_py_calibration.html)

**Note**: An already Hololens calibration has already been performed and the calibration data in json format is [here](output/data.json)

## How to use:
- Prepare a chessboard marker for calibration printing in full scale the [chessboard.pdf](chessboard.pdf)
- Take **at least** 10 picture with your camera and move them inside the _"../CalibImage/"_ directory
- Run the python script
  ```bash
    python main.py
  ```
  If necessary (e.g. you have a different type of chessboard) you can change the following parameters:
  
    | Argument            | Description                             | Required | Default        |
    |---------------------|-----------------------------------------|----------|----------------|
    | `--debug`           | Let you change the debug output folder  | `False`  | `'./output/'`  |
    | `--square_size`     | Chessboard squares size in cm           | `False`  | `3`            |
    | `--pattern_width`   | Chessboard pattern width  cm            | `False`  | `7`            |
    | `--pattern_height`  | Chessboard pattern height in cm         | `False`  | `5`            |

    ```bash
    python Calibration/main.py [--debug <output path>] [--square_size] [--pattern_width] [--pattern_height] [<image mask>]
    ```
    E.g. run with the following command:
    ```bash
    python Calibration/main.py --debug output --square_size 3 --pattern_width 7 --pattern_height 5 CalibImage/*.jpg
    ```
- The script will create a _.json_ and _.yaml_ files inside "../output/" folder which contains all the intrinsic parameters of the camera 
