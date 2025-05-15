## Lesson 0: Installation

## Windows Installation Instructions

Drake is not supported by Windows. However, you can run Drake using [WSL](https://learn.microsoft.com/en-us/windows/wsl/install). If you haven't already, install WSL using the link above. (You will need admin access so you can open up powershell in administrative mode).

While still in your powershell window, install Ubuntu 24.04 if possible, but Ubuntu 22.04 should also work.
```powershell
wsl --install -d Ubuntu-24.04
```
This will take a few minutes to complete.

Next, open VSCode and install the WSL extension from Microsoft.

Then, open the command pallet (`Ctrl+Shift+P`) and select `WSL: Connect to WSL in Distro using New Window`. Select `Ubuntu-24.04` and continue. Then, you can go ahead and continue as normal with the Linux/MacOS install instructions.

## Linux/MacOS Installation Instructions

### Step 1 - Clone the Tutorial Repository

First, clone this repository. On a terminal on your computer, navigate to a good place for this repository to exist, and run the following command to clone the repo.

```
git clone git@github.com:logdog/drake-tutorials.git
```

Open this repo in VSCode, making sure that it is the only folder open (check the Explorer tab). 

### Step 2 - Create Virtual Environment

Next, create a .venv virtual environment by pressing Ctrl+Shift+P, create the environment, select `.venv` (not conda) and a good base path for python (I chose version 3.12). Open a new terminal (Ctrl+~) and it should automatically source the environment. Your terminal should look like the following:

```
(.venv) ldihel@hyunlab-xps16:~/dev/drake-tutorials$ 
```

If you type in the command `which pip` into your environment, it should be in `.venv/bin/pip`. If this doesn't work, then manually run the command
```
source .venv/bin/activate
```
to source your environment. Then, try `which pip` again.

### Step 3 - Install Drake with pip

Install drake and its dependencies.

```
pip install drake
```

### Step 4 - Verification

Verify that your installation was successful. 

```
python -c "import pydrake; print(pydrake)"
```

Should print `<module 'pydrake' from '/home/ldihel/dev/drake-tutorials/.venv/lib/python3.12/site-packages/pydrake/__init__.py'>`.

At this point, try to run `Lesson-1-solution.py` and see if it works! 
> Note: for windows users, there might be an issue using `matplotlib`. See the troubleshooting section for help.

## Troubleshooting

There might be problems displaying `matplotlib.pyplot` for Windows users because of a lack of support for interactive GUI windows. 

As a workaround, you can use Juypter Notebook to run the script. With the correct Python script open, open up the command pallette (`Ctrl+Shift+P`) and run
```
>Jupyter: Run Current File in Interactive Window
```
You may need to install the IPynotebook.
This will run the script and all of the matplotlib figures will be displayed in a window to the right.

### Next Steps

[Lesson 1](../Lesson-1/)
