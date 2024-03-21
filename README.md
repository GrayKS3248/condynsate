# condynsate

 [condynsate](https://github.com/GrayKS3248/condynsate) is a dynamic system simulation and visualization tool built with [PyBullet](https://pybullet.org/wordpress/) and [MeshCat](https://github.com/meshcat-dev/meshcat-python). It automatically simulates multiple objects and their interactions as described by [.urdf](http://wiki.ros.org/urdf), [.stl](https://en.wikipedia.org/wiki/STL_(file_format)), and [.obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file) files and can render real time visualizations in a web browser.

# Installation

## From Source

condynsate requires python => 3.8

```bash
pip install condynsate
```

## Miniconda

* **Install Miniconda**
  
  * [Install Miniconda.](https://docs.conda.io/projects/miniconda/en/latest/)
  
  * To verify that Miniconda is installed properly, open **Anaconda Prompt (Miniconda3)** and run the command:
    
    ```bash
    conda --version
    ```

* **Create a new virtual environment**
  
  * To create a new environment, open **Anaconda Prompt (Miniconda3)** and run the command:
    
    ```bash
    conda create -n ENVNAME
    ```
  
  * Next, activate the newly made environment: 
    
    ```bash
    conda activate ENVNAME
    ```
  
  * You can confirm the virtual environment is activated when environment name on the left side of the command line changes from ``(base)`` to``(ENVNAME)``

* **Install dependencies from Conda-Forge:**
  
  * Configure the environment channels by running the commands:
    
    ```bash
    conda config --env --add channels conda-forge
    conda config --env --set channel_priority strict
    conda clean -a -i
    ```
  
  * Install dependencies available from Conda-Forge by running the command:
    
    ```bash
    conda install -y python=3 numpy pynput matplotlib pybullet control sympy notebook
    ```

* **Install condynsate using pip**
  
  * To install condynsate run the commands:
    
    ```bash
    pip cache purge
    pip install condynsate
    ```
  
  * You can ensure condynsate installed correctly by starting a python bash, importing condynsate, and checking the version number:
    
    ```python
    python
    import condynsate
    condynsate.__version__ 
    ```

# Usage

For examples of usage, see [examples](https://github.com/w-chang/ae353-sp24/tree/main/Projects).