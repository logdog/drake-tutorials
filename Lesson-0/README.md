## Lesson 0: Installation

### Step 1

First, clone this repository. On a terminal on your computer, navigate to a good place for this repository to exist, and run the following command to clone the repo.

```
git clone git@github.com:logdog/drake-tutorials.git
```

Open this repo in VSCode, making sure that it is the only folder open (check the Explorer tab). 

### Step 2

Next, create a .venv virtual environment by pressing Ctrl+Shift+P, create the environment, select `.venv` (not conda) and a good base path for python (I chose version 3.12). Open a new terminal (Ctrl+~) and it should automatically source the environment. Your terminal should look like the following:

```
(.venv) ldihel@hyunlab-xps16:~/dev/drake-tutorials$ 
```

If you type in the command `which pip` into your environment, it should be in `.venv/bin/pip`.

### Step 3

Install drake, matplotlib, and numpy.

```
pip install drake matplotlib numpy
```

### Step 4

Verify that your installation was successful. 

```
python -c "import pydrake; print(pydrake)"
```

Should print `<module 'pydrake' from '/home/ldihel/dev/drake-tutorials/.venv/lib/python3.12/site-packages/pydrake/__init__.py'>`.

### Next Steps

[Lesson 1](../Lesson-1/)