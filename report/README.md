# Latex template
This is a template for writing reports at AAU. 

The advantages of doing it this way: 
- The latex files are compiled locally on the computer, giving a significant speedup in compile time.
- Since the files are shared using git, if one of your group members fucks up it is only on their computer, and you can still compile. Therefore Please ensure the project can compile before pushing.
- You can edit the files in your favorite text editor we have included setup guide for IntelliJ and VS Code.
- With the texteditor comes tools such as GitHub copilot which can eliminate a lot of the boring stuff.
- 

## Installing Tex
There are several different Tex distributions to choose from for compiling tex. 
We recomend using Tex Live since it comes with all packages which minimizes all the trouble for you. 
A guide to how to install Tex Live can be found [here](https://www.tug.org/texlive/)

Alternatively install MikTex. A guide can be found [here](https://miktex.org/download)

## Installing Python
Python is used for creating all the figures in the report. Therefore python needs to be installed. 
There are several different ways to do it, Google is your friend ;-).



## IntelliJ Idea
The IntelliJ Idea IDE is the recomended IDE for writing the report. 
The jetbrains tools are provided by AAU. Please make sure you have installed the Jetbrains toolbox and IntelliJ Idea Ultimate before proceeding. 

### Clone the repo
First download the repository. 

### Setup the working evironement
Then setup the right SDK, by pressing the gear in the to right corner, and then pressing project structure. 
Under project settings and project. 
Press the dropdown menu SDK: 
Choose "Add SDK" 
Add python SDK
Select new environment
Base interpreter should be your install of python and location should be inside the project in a directory called venv

Then to activate the virtual environment you just created open a terminal (Alt+F12) and run the following command for Windows: 

  ```bash
  source venv/Scripts/activate
  ```
and on linux: 
  ```bash
  source venv/bin/activate
  ```

Then run the command: 

  ```bash
  pip install -r requirements.txt
  ```
  
If you would like to use some other python packages for creating the figures, simply add them to the requirements.txt and rerun the command. That way the rest of the group also gets the requirements when you push the next time. 

## Comipilation
To compile the pdf press and run the 'Main PDF' run configuration in the top right of the screen.

## Visual Studio Code
If you like vscode better it is also an option. 
Make sure VSCode is installed. 


### Clone the repo
First download the repository. 

### Installing extentions
VSCode automatically promts you to install the recomended extentions. 

### Setup the working environement
Open the Command Palette by pressing ```(Ctrl + Shift + P)```
The search for **Python: Create Environment**
Select **Venv**
Then select the base interpreter of your choise. 
For the requirements choose "requirements.txt"

## Compilation
To compile the PDF press ```(Ctrl + Shift + P)``` and type 'LaTeX Workshop: Build with recipe' and choose 'Main PDF'

Then you should be all set up to start cranking out report. 


## Extentions 
Both for IntelliJ and VS Code there are several different extenstions which greatly enhances the experience. 
- GitHub Copilot
- TodoTree

A feature which is unfortunately not included is comments. The alternative feature is using TODO's. By making a comment and starting it by "TODO:" the extention TodoTree will automatically find all these comments and make it easier to get an overview. 

# Structure
This template has already created a structure for you to use. Feel free to modify it to your needs. 
## Chapters 
Chapters are put in the 'chapters' folder. Each chapter has a number preceding it, this is done to ensure the chapters show up in the same order as in the report.

Each chapter is a folder containing atleast two different things: 
- The 'figures' folder
- The meta.tex file

## The figures folder
The figures folder contains all the python scripts which creates the beautiful figures for your report. 

## The meta.tex file
The meta.tex file is where all the sections of the chapter is put in. The reason for the meta.tex file is to make it easier to move the different sections around and to reduce the risk of merge conflicts. 

The meta.tex should also include the meta text/ introduction for the chapter. 

# Tables
Some macros has also been included to make tables easier to make and make them uniform. An example can be seen in the chapters\03Development\01Test.tex file. 

The macros are: 
- \toprule 
- \midrule
- \bottomrule

## \toprule 
The \toprule should be put in at the start of the table, before the heading. 
## \midrule
\midrule should be put after each new row in the table. 
## \bottomrule
Should be put after the last row in the table. 
