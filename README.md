# condynsate

## Table of Contents

- [**Introduction**](#introduction)

- [**Windows Usage**](#Windows-Usage)
  
  - [Windows Command Line Basics](#Windows-Command-Line-Basics)
    
    - [How to open an Anaconda Prompt in Windows](#How-to-open-an-Anaconda-Prompt-in-Windows)
    
    - [How to run a command in Windows](#How-to-run-a-command-in-Windows)
    
    - [How to change the working directory in Windows](#How-to-change-the-working-directory-in-Windows)
  
  - [Windows Installation](#Windows-Installation)
    
    - [1 Install Miniconda for Windows](#1-Install-Miniconda-for-Windows)
    
    - [2 Install condynsate in a Miniconda virtual environment in Windows](#2-Install-condynsate-in-a-Miniconda-virtual-environment-in-Windows)
    
    - [3 Clone the ae353-sp24 Repository using Git in Windows](#3-Clone-the-ae353-sp24-Repository-using-Git-in-Windows)
  
  - [Running Projects in Windows](#Running-Projects-in-Windows)
    
    - [1 Change your working directory in Windows](#1-Change-your-working-directory-in-Windows)
    
    - [2 Get the latest version of the code in Windows](#2-Get-the-latest-version-of-the-code-in-Windows)
    
    - [3 Start a Jupyter Notebook in Windows](#3-Start-a-Jupyter-Notebook-in-Windows)

- [**Linux Usage**](#Linux-Usage)
  
  - [Linux Command Line Basics](#Linux-Command-Line-Basics)
    
    - [How to open the Terminal in Linux](#How-to-open-the-Terminal-in-Linux)
    
    - [How to run a command in Linux](#How-to-run-a-command-in-Linux)
    
    - [How to change the working directory in Linux](#How-to-change-the-working-directory-in-Linux)
  
  - [Linux Installation](#Linux-Installation)
    
    - [1 Clone the ae353-sp24 Repository using Git in Linux](#1-Clone-the-ae353-sp24-Repository-using-Git-in-Linux)
    
    - [2 Install Miniconda for Linux](#2-Install-Miniconda-for-Linux)
    
    - [3 Install condynsate in a Miniconda virtual environment in Linux](#3-Install-condynsate-in-a-Miniconda-virtual-environment-in-Linux)
  
  - [Running Projects in Linux](#Running-Projects-in-Linux)
    
    - [1 Change your working directory in Linux](#1-Change-your-working-directory-in-Linux)
    
    - [2 Get the latest version of the code in Linux](#2-Get-the-latest-version-of-the-code-in-Linux)
    
    - [3 Activate your Conda environment in Linux](#3-Activate-your-Conda-environment-in-Linux)
    
    - [4 Start a Jupyter Notebook in Linux](#4-Start-a-Jupyter-Notebook-in-Linux)

- [**MacOS Usage**](#MacOS-Usage)
  
  - [MacOS Command Line Basics](#MacOS-Command-Line-Basics)
    
    - [How to open the Terminal in MacOS](#How-to-open-the-Terminal-in-MacOS)
    
    - [How to run a command in MacOS](#How-to-run-a-command-in-MacOS)
    
    - [How to change the working directory in MacOS](#How-to-change-the-working-directory-in-MacOS)
  
  - [MacOS Installation](#MacOS-Installation)
    
    - [1 Install xcode select for MacOS](#1-Install-xcode-select-for-MacOS)
    
    - [2 Install Miniconda for MacOS](#2-Install-Miniconda-for-MacOS)
    
    - [3 Grant Input Monitoring Access to Terminal in MacOS](#3-Grant-Input-Monitoring-Access-to-Terminal-in-MacOS)
    
    - [4 Install condynsate in a Miniconda virtual environment in MacOS](#4-Install-condynsate-in-a-Miniconda-virtual-environment-in-MacOS)
    
    - [5 Clone the ae353-sp24 Repository using Git in MacOS](#5-Clone-the-ae353-sp24-Repository-using-Git-in-MacOS)
  
  - [Running Projects in MacOS](#Running-Projects-in-MacOS)
    
    - [1 Change your working directory in MacOS](#1-Change-your-working-directory-in-MacOS)
    
    - [2 Get the latest version of the code in MacOS](#2-Get-the-latest-version-of-the-code-in-MacOS)
    
    - [3 Start a Jupyter Notebook in MacOS](#3-Start-a-Jupyter-Notebook-in-MacOS)

- [**Examples and Tutorials**](#Examples-and-Tutorials)

## Introduction

[condynsate](https://github.com/GrayKS3248/condynsate) is a dynamic system simulation and visualization tool built with [PyBullet](https://pybullet.org/wordpress/) and [MeshCat](https://github.com/meshcat-dev/meshcat-python). It automatically simulates multiple objects and their interactions as described by [.urdf](http://wiki.ros.org/urdf), [.stl](https://en.wikipedia.org/wiki/STL_(file_format)), and [.obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file) files and can render real time visualizations in a web browser.

This tool was built by [Grayson Schaer](http://bretl.csl.illinois.edu/people) during the 2023 Fall semester at the University of Illinois at Urbana-Champaign under the Grainger College of Engineering 2023-24 Strategic Instructional Innovations Program: Computational Tools for Dynamics and Control [grant](https://ae3.engineering.illinois.edu/files/2023/09/UIUC-SIIP-projects-2023-24.FINAL_-1.pdf).

Condynsate provides real-time simulation and visualization of articulated bodies with nonlinear dynamics in Python. It was optimized for education and promotes easy in-class demonstrations and lab demos without physical equipment. Further, it encourages students to activate motor neurons, and simulates play during the learning process --both of which are beneficial to the processes of [memory and learning](https://doi.org/10.3390/educsci13070747).

## Windows Usage

### Windows Command Line Basics

#### How to open an Anaconda Prompt in Windows

When we say "open an Anaconda Prompt," what we mean is to start the **Anaconda Prompt (Miniconda3)** application. Here is one way to do that:

- Click the Windows search bar near the bottom-left corner of your desktop.
- Type "anaconda" into the search field.
- Click **Anaconda Prompt (Miniconda3)**.

Note that this application will only exist after you follow the instructions to [1 Install Miniconda for Windows](1-Install-Miniconda-for-Windows).

#### How to run a command in Windows

When we say "run a command," what we mean is to type something into the window of an Anaconda Prompt and press enter. See [this beginners guide](https://www.makeuseof.com/tag/a-beginners-guide-to-the-windows-command-line/) to [Windows Commands](https://docs.microsoft.com/en-us/windows-server/administration/windows-commands/windows-commands).

#### How to change the working directory in Windows

All the files on your computer are organized in folders, which are commonly referred to as "directories." When you are working on the command line, you are working in one of these directories. Commands you run can find files in that directory, but cannot (by default) find files in other directories.

When we say "change the working directory," we mean exactly that --- telling an Anaconda Prompt the directory in which you want to work.

To do this, we run the command

```
cd path\to\directory
```

where "`path\to\directory`" is replaced by the location of the directory in which you want to work. One easy way way to find this location (i.e., the "path" to your directory) is by dragging its folder from the File Explorer into your Anaconda Prompt window (see documentation on [Quickly Copy Files Paths to Your Command Prompt via Drag and Drop](https://lifehacker.com/quickly-copy-file-paths-to-your-command-prompt-via-drag-5382503). In particular, I would first type "`cd` " (note the single trailing space):

```
C:\Users\jakek>cd 
```

Then, I would drag a folder into the powershell window and press enter. For instance, suppose I had created a folder called `ae353-sp24` somewhere on my computer and dragged it in, then pressed enter --- I would see something like this:

```
C:\Users\jakek>cd C:\Users\jakek\OneDrive\Documents\ae353-sp23
C:\Users\jakek\OneDrive\Documents\ae353-sp23>
```

See documentation on [Find and Open Files using Windows Command Prompt](https://www.faqforge.com/windows/windows-10/find-and-open-files-using-windows-command-prompt/) for a way to search for the directory location of files on your computer.

### Windows Installation

#### 1 Install Miniconda for Windows

- **If you already have Miniconda installed, you can ignore this step.**

- **If you have some other distribution of conda installed (Anaconda, Miniforge, etc.), we reccomend that you first uninstall that distribution then install Miniconda.**

If you do not have Miniconda already installed, download the most recent Miniconda installer for Windows 64 from the [miniconda website](https://docs.conda.io/projects/miniconda/en/latest/) or just click [here](https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe) to download it automatically. Once the .exe is downloaded, run it. The .exe file has the format Miniconda3-latest-Windows-x86_64 and will be in your default downloads folder C:\Users\USER NAME\Downloads.

Once the wizard is running, click **next**.

![conda1](https://github.com/w-chang/ae353-sp24/blob/main/Images/conda_1.png?raw=true)

Click **I Agree**.

![conda2](https://github.com/w-chang/ae353-sp24/blob/main/Images/conda_2.png?raw=true)

Ensure **Just Me (recommended)** is selected and click **Next**.

![conda3](https://github.com/w-chang/ae353-sp24/blob/main/Images/conda_3.png?raw=true)

Click **Next**.

![conda4](https://github.com/w-chang/ae353-sp24/blob/main/Images/conda_4.png?raw=true)

Ensure **Create start menu shortcuts (supported packages only)** is checked and **Register Miniconda3 as my default Python 3.11** is ***NOT CHECKED***.

![conda5](https://github.com/w-chang/ae353-sp24/blob/main/Images/conda_5.png?raw=true)

Once installation is complete, click **next**.

![conda6](https://github.com/w-chang/ae353-sp24/blob/main/Images/conda_6.png?raw=true)

You may deselected **Getting started with Conda** and **Welcome to Anaconda** then click **Finish**.

![conda7](https://github.com/w-chang/ae353-sp24/blob/main/Images/conda_7.png?raw=true)

Navigate to the Windows Start Menu folder located at **C:\Users\USER NAME\AppData\Roaming\Microsoft\Windows\Start Menu\Programs\Miniconda3 (64-bit)**.

![conda8](https://github.com/w-chang/ae353-sp24/blob/main/Images/conda_8.png?raw=true)

To create a desktop shortcut to Anaconda Prompt (the application that will be used interact with Miniconda), right click on **Anaconda Prompt (Miniconda3)**, navigate to **Send to**, then click **Desktop (create shortcut)**.

![conda9](https://github.com/w-chang/ae353-sp24/blob/main/Images/conda_9.png?raw=true)

To verify that Miniconda is installed properly, open **Anaconda Prompt (Miniconda3)** and run the command

```bash
conda --version
```

If Miniconda is installed correctly, you should get a response that lists the version of Miniconda you just installed.

#### 2 Install condynsate in a Miniconda virtual environment in Windows

To install condynsate in a Miniconda virtual environment, open **Anaconda Prompt (Miniconda3)** and run the command

```bash
conda create -n ae353
```

When prompted, type `y` and press enter.

When complete, run the command

```bash
conda activate ae353
```

You can confirm the virtual environment you just created is activated correctly when the text `(base)` on the left hand side of the command line changes to `(ae353)`.

Once you've ensured the virtual environment is activated, configure it by running the commands

```bash
conda config --env --add channels conda-forge
conda config --env --set channel_priority strict
conda clean -a -y
```

Now install the Conda-Forge dependencies by running the command

```bash
conda install -y python=3 git control matplotlib notebook numpy pybullet pynput pyside6 sympy
```

Once the installation is complete, install condynsate by running the commands

```bash
pip cache purge
pip install condynsate
```

You can check that condynsate installed correctly by running the command

```bash
python
```

This starts a Python shell in your Anaconda prompt. Next run the commands

```python
import condynsate
condynsate.__version__
```

If condynsate is installed correctly, the current version will be shown. You may now type

```python
quit()
```

to quit the Python shell.

#### 3 Clone the ae353-sp24 Repository using Git in Windows

In your already opened **Anaconda Prompt (Miniconda3)** navigate to the directory you want to clone the ae353-sp24 code repository into. This is done using the [change directory command](#How to change the working directory).

Next, clone the repository into the current directory by running the command

```bash
git clone https://github.com/w-chang/ae353-sp24.git
```

You may need to select your credential manager. Select "*manager*" from the popup window and then log into github.com using your github account.

The ae353-sp24 code repository is now cloned to your machine. You will find all design projects in this repository.

### Running Projects in Windows

#### 1 Change your working directory in Windows

Open **Anaconda Prompt (Miniconda3)** and run the command

```
conda activate ae353
```

You should see the prefix to your prompt change from `(base)` to `(ae353)`.

Next, change your working directory to wherever you cloned this repository.

#### 2 Get the latest version of the code in Windows

In your already open **Anaconda Prompt (Miniconda3)** run the commands

```
git fetch
git pull
```

Do not worry, this will not overwrite any of your own work. If you see any errors or warnings, post a note to [Piazza](https://piazza.com/) and course staff will help resolve them.

#### 3 Start a Jupyter Notebook in Windows

In your already open **Anaconda Prompt (miniconda3)** run the command

```
jupyter notebook
```

A browser window should open with the Jupyter Notebook interface. You can now navigate to and open any of the notebooks (with extension `.ipynb`) used for in-class examples or for design projects.

**We strongly recommend you duplicate and work with a copy of any given notebook rather than working with the original.** Feel free to ignore this suggestion if you are a `git` expert.

## Linux Usage

### Linux Command Line Basics

#### How to open the Terminal in Linux

When we say "open the terminal", what we mean is to start the **Terminal** application. To do this, you can use the keyboard shortcut **ctrl+alt+t**.

#### How to run a command in Linux

When we say "run a command," what we mean is to type something into the window of the Terminal and press enter.

#### How to change the working directory in Linux

All the files on your computer are organized in folders, which are commonly referred to as "directories." When you are working on the command line, you are working in one of these directories. Commands you run can find files in that directory, but cannot (by default) find files in other directories.

When we say "change the working directory," we mean exactly that --- telling the Terminal the directory in which you want to work.

To move in the directory tree, we run the command

```
cd path\to\directory
```

where "`path\to\directory`" is replaced by the location of the directory in which you want to work. If you want to move up a single directory, you can run the command

```
cd ..
```

### Linux Installation

#### 1 Clone the ae353-sp24 Repository using Git in Linux

Open the terminal. Navigate to the directory you want to clone the ae353-sp24 code repository into. Now, clone the repository into the current directory by typing the command

```bash
git clone https://github.com/w-chang/ae353-sp24.git
```

The ae353-sp24 code repository is now cloned to your machine. You will find all design projects in this repository.

#### 2 Install Miniconda for Linux

- **If you already have Miniconda installed, you can ignore this step.**

- **If you have some other distribution of conda installed (Anaconda, Miniforge, etc.), we reccomend that you first uninstall that distribution then install Miniconda.**

If you do not have Miniconda already installed, run these four commands in the Terminal to quickly and quietly install the latest 64-bit version of the installer.

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
```

**Restart the Terminal.** After installing, initialize your newly-installed Miniconda by running the command

```bash
~/miniconda3/bin/conda init bash
```

To verify that Miniconda is installed properly, run the command

```bash
conda --version
```

You should get a response that lists the version of Miniconda you just installed. If you do not, more help on installation can be found [here](https://docs.conda.io/projects/miniconda/en/latest/).

#### 3 Install condynsate in a Miniconda virtual environment in Linux

To install condynsate in a Miniconda virtual environment, open the Terminal and run the command

```bash
conda create -n ae353
```

When prompted, type `y` and press enter. This creates a new virtual environment. to activate the environment, run the command

```bash
conda activate ae353
```

You can confirm the virtual environment you just created is activated correctly when the text `(base)` on the left hand side of the command line changes to `(ae353)`.

Once you've ensured the virtual environment is activated, configure it by running the commands

```bash
conda config --env --add channels conda-forge
conda config --env --set channel_priority strict
conda clean -a -y
```

Now install the Conda-Forge dependencies by running the command

```bash
conda install -y python=3 git control matplotlib notebook numpy pybullet pynput pyside6 sympy
```

Once the installation is complete, install condynsate by running the commands

```bash
pip cache purge
pip install condynsate
```

You can check that condynsate installed correctly by running the command

```bash
python3
```

This starts a Python shell in your Anaconda prompt. Next run the commands

```python
import condynsate
condynsate.__version__
```

If condynsate is installed correctly, the current version will be shown. You may now type

```python
quit()
```

to quit the Python shell and exit out of the Terminal.

### Running Projects in Linux

#### 1 Change your working directory in Linux

Open the Terminal and change your working directory to wherever you cloned this repository.

#### 2 Get the latest version of the code in Linux

Run the commands

```
git fetch
git pull
```

Do not worry, this will not overwrite any of your own work. If you see any errors or warnings, post a note to [Piazza](https://piazza.com/) and course staff will help resolve them.

#### 3 Activate your Conda environment in Linux

Run the command

```
conda activate ae353
```

You should see the prefix to your prompt change from `(base)` to `(ae353)`. This means you are in the Conda environment you created for work with AE353.

#### 4 Start a Jupyter Notebook in Linux

Run the command

```
jupyter notebook
```

A browser window should open with the Jupyter Notebook interface. You can now navigate to and open any of the notebooks (with extension `.ipynb`) used for in-class examples or for design projects.

**We strongly recommend you duplicate and work with a copy of any given notebook rather than working with the original.** Feel free to ignore this suggestion if you are a `git` expert.

## MacOS Usage

### MacOS Command Line Basics

#### How to open the Terminal in MacOS

When we say "open a terminal," what we mean is to start the **Terminal** application. Here are two ways to do that:

- Click the Launchpad icon ![img](https://help.apple.com/assets/63FFD63D71728623E706DB4F/63FFD63E71728623E706DB56/en_US/a1f94c9ca0de21571b88a8bf9aef36b8.png) in the Dock, type Terminal in the search field, then click Terminal.
- In the Finder ![img](https://help.apple.com/assets/63FFD63D71728623E706DB4F/63FFD63E71728623E706DB56/en_US/058e4af8e726290f491044219d2eee73.png), open the /Applications/Utilities folder, then double-click Terminal.

See documentation on [Open Terminal](https://support.apple.com/guide/terminal/open-or-quit-terminal-apd5265185d-f365-44cb-8b09-71a064a42125/mac) for more information. Note that it is often helpful to have more than one terminal window open at the same time (or more than one tab in the same window).

#### How to run a command in MacOS

When we say "run a command," what we mean is to type something into the Terminal and press return. For example, suppose we said:

> run the command `pwd` to find your current working directory

You would type `pwd` into the terminal window and press return, with the result being something like this:

```bash
timothybretl@Timothys-MacBook-Pro ~ % pwd
/Users/timothybretl
```

See documentation on [Execute commands and run tools in Terminal on Mac for more information](https://support.apple.com/guide/terminal/execute-commands-and-run-tools-apdb66b5242-0d18-49fc-9c47-a2498b7c91d5/mac). Also see the [Command Line Primer](https://developer.apple.com/library/archive/documentation/OpenSource/Conceptual/ShellScripting/CommandLInePrimer/CommandLine.html) for a list of frequently used commands.

#### How to change the working directory in MacOS

All the files on your computer are organized in folders, which are commonly referred to as "directories." When you are working on the command line in a terminal, you are working in one of these directories. Commands you run can find files in that directory, but cannot (by default) find files in other directories.

When we say "change the working directory," we mean exactly that --- telling the terminal the directory in which you want to work.

To do this, we run the command

```bash
cd path/to/directory
```

where "`path/to/directory`" is replaced by the location of the directory in which you want to work. One easy way way to find this location (i.e., the "path" to your directory) is by dragging its folder from the Finder into your terminal window (see documentation on [Drag items into a Terminal window on Mac](https://support.apple.com/guide/terminal/drag-items-into-a-terminal-window-trml106/mac)). In particular, I would first type "`cd` " (note the single trailing space):

```bash
timothybretl@Timothys-MacBook-Pro ~ % cd 
```

Then, I would drag a folder into the terminal window and press return. For instance, suppose I had created a folder called `ae353-sp24` somewhere on my computer and dragged it in, then pressed return --- I would see something like this:

```bash
timothybretl@Timothys-MacBook-Pro ~ % cd /Users/timothybretl/Documents/ae353-sp24
timothybretl@Timothys-MacBook-Pro ae353-sp24 %
```

See documentation on [Specify files and folders in Terminal on Mac](https://support.apple.com/guide/terminal/specify-files-and-folders-apd3cf6fe02-3ec8-48f1-951f-866e52955fc8/mac) for other ways to specify the path to a directory.

### MacOS Installation

#### 1 Install xcode select for MacOS

If you do not already have Command Line Tools installed, open the Terminal and run the command

```Bash
xcode-select --install
```

Click the Install button on the window that opens and agree to the license agreement.

#### 2 Install Miniconda for MacOS

- **Make sure your machine's operating system is updated to the most current version.**

- **The installation steps for M1 machines vs. x86_86 machines are different.**

- **If you have some other distribution of conda installed (Anaconda, Miniforge, etc.), we reccomend that you first uninstall that distribution then install Miniconda.**

##### ARM64 archtecture (M1)

- Open the Terminal.

- Run these four commands to quickly and quietly install the latest 64-bit version of the **ARM64** installer on your **M1** machine.

```bash
mkdir -p ~/miniconda3
curl https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh -o ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
```

- Restart the terminal.

- Initialize your newly-installed Miniconda by running the commands

```bash
~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
```

- To verify that Miniconda is installed properly, run the command

```bash
conda --version
```

- You should get a response that lists the version of Miniconda you just installed. If you do not, more help on installation can be found [here](https://docs.conda.io/projects/miniconda/en/latest/).

##### x86_64 archtecture (Intel)

- Open the Terminal.

- Run these four commands to quickly and quietly install the latest 64-bit version of the **x86_64** installer on your **Intel** machine.

```bash
mkdir -p ~/miniconda3
curl https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-x86_64.sh -o ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
```

- Restart the terminal.

- Initialize your newly-installed Miniconda by running the commands

```bash
~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
```

- To verify that Miniconda is installed properly, run the command

```bash
conda --version
```

- You should get a response that lists the version of Miniconda you just installed. If you do not, more help on installation can be found [here](https://docs.conda.io/projects/miniconda/en/latest/).

#### 3 Grant Input Monitoring Access to Terminal in MacOS

To use keyboard interactivity in the projects, you must first grant the Terminal security access to monitor inputs (keyboards and mice). To do this

- Choose Apple menu  > System Settings

- Click Privacy & Security ![](https://help.apple.com/assets/6529D8627783ACA29F083601/6529D866CFDD5FD5B90BAB1B/en_US/f9979df145e31ea9fb18995403d2b2f6.png) (or Security & Privacy on older systems).

- In the ribbon on the side, click the Input Monitoring tab. If the terminal is not in the list you will need to add it:
  
  - Click the lock icon to unlock the list. Input your password to unlock.
  
  - Click the **+** button.
  
  - Navigate to *Applications/Utilities*.
  
  - Locate the Terminal app and select **Open** to add it to the Input Monitoring list

- Check the box next to the Terminal app in the Input Monitoring list to enable it.

- Now, in the ribbon in the side, click the Accessibility tab. If the Terminal is not in the list, follow the steps above to add it.

- Check the box next to the Terminal app in the Accessibility list to enable it.

#### 4 Install condynsate in a Miniconda virtual environment in MacOS

To create a new Miniconda virtual environment, in your already open Terminal run the command

```bash
conda create -n ae353
```

When prompted, type `y` and press enter. This creates a new virtual environment. To activate the environment, run the command

```bash
conda activate ae353
```

You can confirm the virtual environment you just created is activated correctly when the text `(base)` on the left hand side of the command line changes to `(ae353)`.

Once you've ensured the virtual environment is activated, configure it by running the commands

```bash
conda config --env --add channels conda-forge
conda config --env --set channel_priority strict
conda clean -a -y
```

Now install pip from Conda-Forge by running the command

```bash
conda install pip
```

Once the installation is complete, install condynsate and other dependencies via pip by running the commands

```bash
pip cache purge
pip install wheel condynsate pyside6
```

You can check that condynsate installed correctly by running the command

```bash
python3
```

This starts a Python shell in your Terminal. Next run the commands

```python
import condynsate
condynsate.__version__
```

If condynsate is installed correctly, the current version will be shown. You may now type

```python
quit()
```

to quit the Python shell.

#### 5 Clone the ae353-sp24 Repository using Git in MacOS

In your already opened Terminal, navigate to the directory you want to clone the ae353-sp24 code repository into. Now, clone the repository into the current directory by typing the command

```bash
git clone https://github.com/w-chang/ae353-sp24.git
```

The ae353-sp24 code repository is now cloned to your machine. You will find all design projects in this repository.

### Running Projects in MacOS

#### 1 Change your working directory in MacOS

Open the Terminal and run the command

```
conda activate ae353
```

You should see the prefix to your prompt change from `(base)` to `(ae353)`.

Next, change your working directory to wherever you cloned this repository.

#### 2 Get the latest version of the code in MacOS

In your already open Terminal run the commands

```
git fetch
git pull
```

Do not worry, this will not overwrite any of your own work. If you see any errors or warnings, post a note to [Piazza](https://piazza.com/) and course staff will help resolve them.

#### 3 Start a Jupyter Notebook in MacOS

In your already open Terminal run the command

```
jupyter notebook
```

A browser window should open with the Jupyter Notebook interface. You can now navigate to and open any of the notebooks (with extension `.ipynb`) used for in-class examples or for design projects.

**We strongly recommend you duplicate and work with a copy of any given notebook rather than working with the original.** Feel free to ignore this suggestion if you are a `git` expert. 

# Examples and Tutorials

For examples of usage, see [examples](https://github.com/GrayKS3248/condynsate/tree/main/examples). For tutorials on using condynsate to design a project, see [tutorials](https://github.com/GrayKS3248/condynsate/tree/main/tutorials).