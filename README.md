# SMARTGATE PROJECT - PYTHON VERSION
This folder contains all the useful python codes to implement
the smartgate project

## How to use

1. Clone the repository running the following line:\
   (A very useful program to manage git repositories on Windows is [GitHub Desktop](https://desktop.github.com/))
```bash
git clone https://github.com/lorecol/SmartGate.git
```
or simply download the zip.\
Move to the _python_ branch
```bash
git checkout python
```

2. Install _python 3.10.11_ on Windows following [this guide](https://www.python.org/downloads/windows/)\
NOTE: During installation be sure that "_Add python.exe to PATH_" is checked!!\
To check if python is successfully installed open a terminal window and run:
```bash
python --version
```

A useful python editor for windows are [PyCharm](https://www.jetbrains.com/pycharm/download/#section=windows) or 
[VScode](https://code.visualstudio.com/) but you can continue also without installing it.

3. Create a virtual environment: \
Inside the cloned repository directory "_../SmartGate/_" open a terminal window (right click, open in terminal) and create a virtual environment
```bash
python -m venv venv 
```

4. Activate virtual environment:\

```bash
.\venv\Scripts\activate
```

5. Install all the requirements:
```bash
pip install -r requirements.txt 
```

6. Follow the README file of [PointCloud](PointCloud/README.md)
