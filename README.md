# condynsate

 [condynsate](https://github.com/GrayKS3248/condynsate) is a dynamic system simulation and visualization tool built with [PyBullet](https://pybullet.org/wordpress/) and [MeshCat](https://github.com/meshcat-dev/meshcat-python). It automatically simulates multiple objects and their interactions as described by [.urdf](http://wiki.ros.org/urdf), [.stl](https://en.wikipedia.org/wiki/STL_(file_format)), and [.obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file) files and can render real time visualizations in a web browser.



# Installation without Virtual Environment (not recommended)

For a base installation:

```bash
pip install condynsate
```

To include optional packages helpful in education settings:

```bash
pip install condynsate[edu]
```



# Recommended Windows Installation (Miniconda)
## 1. Install Miniconda for Windows
If you do not have Miniconda already installed, download the most recent Miniconda installer for Windows 64 from the [miniconda website](https://docs.conda.io/projects/miniconda/en/latest/) or just click [here](https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe) to download it automatically. Once the .exe is downloaded, run it. The .exe file has the format Miniconda3-latest-Windows-x86_64 and will be in your default downloads folder C:\Users\USER NAME\Downloads. 

* Once the wizard is running, click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_1.png?raw=true" alt="conda_1" width="495" height="375"/>



* Click **I Agree**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_2.png?raw=true" alt="conda_2" width="495" height="375"/>



* Ensure **Just Me (recommended)** is selected and click **Next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_3.png?raw=true" alt="conda_3" width="495" height="375"/>



* Click **Next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_4.png?raw=true" alt="conda_4" width="495" height="375"/>



* Ensure **Create start menu shortcuts (supported packages only)** is checked and **Register Miniconda3 as my default Python 3.11** is ***NOT CHECKED***.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_5.png?raw=true" alt="conda_5" width="495" height="375"/>



* Once installation is complete, click **next**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_6.png?raw=true" alt="conda_6" width="495" height="375"/>



* You may deselected **Getting started with Conda** and **Welcome to Anaconda** then click **Finish**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_7.png?raw=true" alt="conda_7" width="495" height="375"/>



* Navigate to the Windows Start Menu folder located at **C:\Users\USER NAME\AppData\Roaming\Microsoft\Windows\Start Menu\Programs\Miniconda3 (64-bit)**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_8.png?raw=true" alt="conda_8" width="495" height="375"/>



* To create a desktop shortcut to Anaconda Prompt (the application that will be used interact with Miniconda), right click on **Anaconda Prompt (Miniconda3)**, navigate to **Send to**, then click **Desktop (create shortcut)**.

<img src="https://github.com/GrayKS3248/condynsate/blob/main/images/conda_9.png?raw=true" alt="conda_9" width="495" height="375"/>



* To verify that Miniconda is installed properly, open **Anaconda Prompt (Miniconda3)** and type 

```bash
conda --version
```


* You should get a response that lists the version of Miniconda you just installed.

Miniconda is now installed on your machine.



## 2. Install condynsate in a Miniconda virtual environment
To install condynsate in a Miniconda virtual environment, open **Anaconda Prompt (Miniconda3)** and type

```bash
conda create -n condynsate python==3.8.18
```
Press enter.

When prompted, type 

```bash
y
```
then press enter.

When complete, type

```bash
conda activate condynsate
```
then press enter.

You can confirm the virtual environment is activated when 

```bash
(base)
```
changes to 
```bash
(condynsate)
```
at the begining of the prompt line. Once the virtual environment is activated, type
```bash
pip install condynsate[edu]
```
and press enter. This installs condynsate and some other optional dependencies that are helpful when using condynsate.

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

condynsate is now installed on your machine. You may type 

```python
quit()
```
to quit the Python shell and exit out of the Anaconda prompt.



# Recommended Linux Installation (Miniconda)
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
To install condynsate in a Miniconda virtual environment, open the terminal and create a new virtual environment with python 3.8. To open the terminal press **ctrl+alt+t**. When the terminal is open, run the command:
```bash
conda create -n condynsate python==3.8.18
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



# Recommended MacOS Installation (Miniconda)

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



## 2. Install condynsate in a Miniconda virtual environment

To install condynsate in a Miniconda virtual environment, open the terminal and create a new virtual environment with python 3.8. To open the terminal do one of the following:

- Click the Launchpad icon ![img](https://help.apple.com/assets/63FFD63D71728623E706DB4F/63FFD63E71728623E706DB56/en_US/a1f94c9ca0de21571b88a8bf9aef36b8.png) in the Dock, type Terminal in the search field, then click Terminal.
- In the Finder ![img](https://help.apple.com/assets/63FFD63D71728623E706DB4F/63FFD63E71728623E706DB56/en_US/058e4af8e726290f491044219d2eee73.png), open the /Applications/Utilities folder, then double-click Terminal.

When the terminal is open, run the command: 

```bash
conda create -n condynsate python==3.8.18
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