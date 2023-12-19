# condynsate

 [condynsate](https://github.com/GrayKS3248/condynsate) is a dynamic system simulation and visualization tool built with [PyBullet](https://pybullet.org/wordpress/) and [MeshCat](https://github.com/meshcat-dev/meshcat-python). It automatically simulates multiple objects and their interactions as described by [.urdf](http://wiki.ros.org/urdf), [.stl](https://en.wikipedia.org/wiki/STL_(file_format)), and [.obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file) files and can render real time visualizations in a web browser.



# Windows Installation
## 1. Install Git for Windows
If you do not have Git already installed, download the most recent Git installer for Windows 64 from the [git-scm website](https://git-scm.com/download/win) or just click [here](https://github.com/git-for-windows/git/releases/download/v2.43.0.windows.1/Git-2.43.0-64-bit.exe) to download it automatically. Once the .exe is downloaded, run it. The .exe file has the format Git-VERSION-64-bit and will be in your default downloads folder C:\Users\USER NAME\Downloads. Once the wizard is running, click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_1.png?raw=true" alt="git_1" width="495" height="375" align="left"/>



Click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_2.png?raw=true" alt="git_2" width="495" height="375" align="left"/>



Click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_3.png?raw=true" alt="git_3" width="495" height="375" align="left"/>



Click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_4.png?raw=true" alt="git_4" width="495" height="375" align="left"/>



Select **Let Git decide** then click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_5.png?raw=true" alt="git_5" width="495" height="375" align="left"/>



Select **Use Git from Git Bash only** then click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_6.png?raw=true" alt="git_6" width="495" height="375" align="left"/>



Select **Use bundled OpenSSH** then click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_7.png?raw=true" alt="git_7" width="495" height="375" align="left"/>



Select **Use the OpenSSL library** then click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_8.png?raw=true" alt="git_8" width="495" height="375" align="left"/>



Select **Checkout Windows-style, commit Unix-style line endings** then click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_9.png?raw=true" alt="git_9" width="495" height="375" align="left"/>



Select **Use Windows' default console window** then click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_10.png?raw=true" alt="git_10" width="495" height="375" align="left"/>



Select **Fast-forward or merge** then click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_11.png?raw=true" alt="git_11" width="495" height="375" align="left"/>



Select **Git Credential Manager** then click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_12.png?raw=true" alt="git_12" width="495" height="375" align="left"/>



Check only **Enable file system caching** then click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_13.png?raw=true" alt="git_13" width="495" height="375" align="left"/>



Make sure nothing is checked then click **install**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_14.png?raw=true" alt="git_14" width="495" height="375" align="left"/>



Once installation is finished, click **finish**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_15.png?raw=true" alt="git_15" width="495" height="375" align="left"/>



Navigate to **C:\Program Files\Git** in your file explorer.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_16.png?raw=true" alt="git_16" width="495" height="375" align="left"/>



To create a desktop shortcut to git-cmd (the application that will be used to pull and push Git repositories), right click on **git-cmd**, navigate to **Send to**, then click **Desktop (create shortcut)**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_17.png?raw=true" alt="git_17" width="495" height="375" align="left"/>



To verify that Git is installed properly, open **git-cmd** and type 

```bash
Git --version
```
You should get a response that lists the version of Git you just installed.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/git_18.png?raw=true" alt="git_18" width="495" height="375" align="left"/>

Git is now installed on your machine.



## 2. Install Miniconda for Windows
If you do not have Miniconda already installed, download the most recent Miniconda installer for Windows 64 from the [miniconda website](https://docs.conda.io/projects/miniconda/en/latest/) or just click [here](https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe) to download it automatically. Once the .exe is downloaded, run it. The .exe file has the format Miniconda3-latest-Windows-x86_64 and will be in your default downloads folder C:\Users\USER NAME\Downloads. Once the wizard is running, click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_1.png?raw=true" alt="conda_1" width="495" height="375" align="left"/>



Click **I Agree**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_2.png?raw=true" alt="conda_2" width="495" height="375" align="left"/>



Ensure **Just Me (recommended)** is selected and click **Next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_3.png?raw=true" alt="conda_3" width="495" height="375" align="left"/>



Click **Next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_4.png?raw=true" alt="conda_4" width="495" height="375" align="left"/>



Ensure **Create start menu shortcuts (supported packages only)** is checked and **Register Miniconda3 as my default Python 3.11** is ***NOT CHECKED***.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_5.png?raw=true" alt="conda_5" width="495" height="375" align="left"/>



Once installation is complete, click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_6.png?raw=true" alt="conda_6" width="495" height="375" align="left"/>



You may deselected **Getting started with Conda** and **Welcome to Anaconda** then click **Finish**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_7.png?raw=true" alt="conda_7" width="495" height="375" align="left"/>



Navigate to the Windows Start Menu folder located at **C:\Users\USER NAME\AppData\Roaming\Microsoft\Windows\Start Menu\Programs\Miniconda3 (64-bit)**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_8.png?raw=true" alt="conda_8" width="495" height="375" align="left"/>



To create a desktop shortcut to Anaconda Prompt (the application that will be used interact with Miniconda), right click on **Anaconda Prompt (Miniconda3)**, navigate to **Send to**, then click **Desktop (create shortcut)**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_9.png?raw=true" alt="conda_9" width="495" height="375" align="left"/>



To verify that Miniconda is installed properly, open **Anaconda Prompt (Miniconda3)** and type 

```bash
conda --version
```
You should get a response that lists the version of Miniconda you just installed.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_10.png?raw=true" alt="conda_10" width="495" height="375" align="left"/>

Miniconda is now installed on your machine.



## 3. Install condynsate in a Miniconda virtual environment
To install condynsate in a Miniconda virtual environment, open **Anaconda Prompt (Miniconda3)** and type
```bash
conda create -n condynsate python==3.10
```
Press enter.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/condynsate_1.png?raw=true" alt="condynsate_1" width="495" height="375" align="left"/>



When prompted, type 

```bash
y
```
then press enter.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/condynsate_2.png?raw=true" alt="condynsate_2" width="495" height="375" align="left"/>



When complete, type

```bash
conda activate condynsate
```
then press enter.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/condynsate_3.png?raw=true" alt="condynsate_3" width="495" height="375" align="left"/>



You can confirm the virtual environment is activated when 

```
(base)
```
changes to 
```
(condynsate)
```
at the begining of the prompt line. Once the virtual environment is activated, type
```bash
pip install condynsate[edu]
```
and press enter. This installs condynsate and some other optional dependencies that are helpful when using condynsate.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/condynsate_4.png?raw=true" alt="condynsate_4" width="495" height="375" align="left"/>



Once the installation is complete, you can confirm that condynsate has been successfully installed and the virtual environment is set up correctly by typing

```bash
python
```
and pressing enter. This starts a Python shell in your Anaconda prompt. Next type
```python
import condynsate
```
and press enter. Finally type
```python
condynsate.__version__
```
and press enter. If condynsate is installed correctly, the current version will be shown.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/condynsate_5.png?raw=true" alt="condynsate_5" width="495" height="375" align="left"/>



condynsate is now installed on your machine. You may type 

```python
quit()
```
to quit the Python shell and exit out of the Anaconda prompt.



# Linux Installation
## 1. Install Miniconda for Linux
If you do not have Miniconda already installed, run these four commands in the terminal to quickly and quietly install the latest 64-bit version of the installer. To open the terminal press **ctrl+alt+t**.

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
```

**Restart the terminal.** After installing, initialize your newly-installed Miniconda. 

```bash
~/miniconda3/bin/conda init bash
```

To verify that Miniconda is installed properly,  run the command 
```bash
conda --version
```

You should get a response that lists the version of Miniconda you just installed. If you do not, more help on installation can be found [here](https://docs.conda.io/projects/miniconda/en/latest/).



## 2. Install condynsate in a Miniconda virtual environment
To install condynsate in a Miniconda virtual environment, open the terminal and create a new virtual environment with python 3.10. To open the terminal press **ctrl+alt+t**. When the terminal is open, run the command:
```bash
conda create -n condynsate python==3.10
```

When complete activate the new environment
```bash
conda activate condynsate
```
You can confirm the virtual environment is activated when 
```
(base)
```
changes to 
```
(condynsate)
```

Install condynsate with edu dependencies to the environment using the pip package manager: 
```bash
pip install condynsate[edu]
```

Once the installation is complete, you can confirm that condynsate has been successfully installed and the virtual environment is set up correctly by running the following three commands:
```python
python
import condynsate
condynsate.__version__
```
If condynsate is installed correctly, the current version will be shown. condynsate is now installed on your machine. You may type 
```python
quit()
```
to quit the Python shell.



# MacOS Installation

## 1. Install Miniconda for MacOS

If you do not have Miniconda already installed, run these four commands in the terminal to quickly and quietly install the latest 64-bit version of the installer. To open the terminal do one of the following:

- Click the Launchpad icon ![img](https://help.apple.com/assets/63FFD63D71728623E706DB4F/63FFD63E71728623E706DB56/en_US/a1f94c9ca0de21571b88a8bf9aef36b8.png) in the Dock, type Terminal in the search field, then click Terminal.
- In the Finder ![img](https://help.apple.com/assets/63FFD63D71728623E706DB4F/63FFD63E71728623E706DB56/en_US/058e4af8e726290f491044219d2eee73.png), open the /Applications/Utilities folder, then double-click Terminal.

```bash
mkdir -p ~/miniconda3
curl https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh -o ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
```

**Restart the terminal.**  After installing, initialize your newly-installed Miniconda. 

```bash
~/miniconda3/bin/conda init bash
```

To verify that Miniconda is installed properly,  run the command 

```bash
conda --version
```

You should get a response that lists the version of Miniconda you just installed. If you do not, more help on installation can be found [here](https://docs.conda.io/projects/miniconda/en/latest/).



## 2. Install Git for MacOS

If you do not have Git already installed, install it via Homebew. First, open the terminal. To open the terminal do one of the following:

- Click the Launchpad icon ![img](https://help.apple.com/assets/63FFD63D71728623E706DB4F/63FFD63E71728623E706DB56/en_US/a1f94c9ca0de21571b88a8bf9aef36b8.png) in the Dock, type Terminal in the search field, then click Terminal.
- In the Finder ![img](https://help.apple.com/assets/63FFD63D71728623E706DB4F/63FFD63E71728623E706DB56/en_US/058e4af8e726290f491044219d2eee73.png), open the /Applications/Utilities folder, then double-click Terminal.

Ensure that you are in the base conda environment by running the command:

```bash
conda deactivate
```

The left hand side of the prompt line in the terminal should read

```bash
(base)
```

Next, type the command:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

**Restart the terminal.** This will install [Homebrew](https://brew.sh/) on your machine. Homebrew is a package manager for MacOS. Next , install Git via Homebrew. Open a new instanace of the terminal. Ensure that you are in the base conda environment by running the command:

```bash
conda deactivate
```

The left hand side of the prompt line in the terminal should read

```bash
(base)
```

Next, type the command:

```bash
brew install git
```

This installs Git to your system. To verify that Git installed correctly,  run the command 

```bash
git --version
```

You should get a response that lists the version of Git you just installed. If you do not, more help on installation can be found [here](https://git-scm.com/download/mac).



## 3. Install condynsate in a Miniconda virtual environment

To install condynsate in a Miniconda virtual environment, open the terminal and create a new virtual environment with python 3.10. To open the terminal do one of the following:

- Click the Launchpad icon ![img](https://help.apple.com/assets/63FFD63D71728623E706DB4F/63FFD63E71728623E706DB56/en_US/a1f94c9ca0de21571b88a8bf9aef36b8.png) in the Dock, type Terminal in the search field, then click Terminal.
- In the Finder ![img](https://help.apple.com/assets/63FFD63D71728623E706DB4F/63FFD63E71728623E706DB56/en_US/058e4af8e726290f491044219d2eee73.png), open the /Applications/Utilities folder, then double-click Terminal.

When the terminal is open, run the command: 

```bash
conda create -n condynsate python==3.10
```

When complete activate the new environment

```bash
conda activate condynsate
```

You can confirm the virtual environment is activated when 

```
(base)
```

changes to 

```
(condynsate)
```

Install condynsate with edu dependencies to the environment using the pip package manager: 

```bash
pip install condynsate[edu]
```

Once the installation is complete, you can confirm that condynsate has been successfully installed and the virtual environment is set up correctly by running the following three commands:

```python
python
import condynsate
condynsate.__version__
```

If condynsate is installed correctly, the current version will be shown. condynsate is now installed on your machine. You may type 

```python
quit()
```

to quit the Python shell.



# Usage
For examples of usage, see [examples](https://github.com/GrayKS3248/condynsate/tree/main/examples).