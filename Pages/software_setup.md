# F1-Tenth_Duke
## Softwware setup in Ubuntu
### installing Terminator
If you are running Ubuntu 20.04 or later, you can run in the terminal:

```bash
sudo add-apt-repository ppa:mattrose/terminator 

sudo apt-get update 

sudo apt install terminator 
```

If you have any questions, [here](https://github.com/gnome-terminator/terminator/blob/master/INSTALL.md) is a link for terminator installation to follow.

### Installing Visual Studio Code
There are three ways to install visual studio code.

Method 1: Snap

[Snap packages](https://phoenixnap.com/kb/snap-packages) are containerized applications. Open the terminal and run the following command to install the vscode self-contained snap package with the required dependencies:

```bash
sudo snap install –classic code 

code –version
```

Method 2: Ubuntu Software Center

Step 1: Open Ubuntu Software Center
Step 2: Find Visual Studio Code
Step 3: Install vscode

If you have any questions, [here](https://phoenixnap.com/kb/install-vscode-ubuntu) is a link to follow.

### Install Git

Using the [apt package management tool](https://phoenixnap.com/kb/apt-linux) is the easiest way to install Git. 

Step 1: Start by updating the system package index. Launch a terminal window (Ctrl+Alt+T) and run the following command:

```bash
sudo apt update
```

Step 2: Install Git from the default Ubuntu repository by running: 

```bash
sudo apt install git
```
Step 3: Verify the installation and version by running: 

```bash
git --version
```

If you have any question, [here]() is a link to follow.

### Installing GitHub Desktop on Ubuntu (for host computer)

Please run the following command in your terminal window: 

```bash
sudo wget https://github.com/shiftkey/desktop/releases/download/release-3.3.1-linux1/GitHubDesktop-linux-amd64-3.3.1-linux1.deb 

sudo apt install ./GitHubDesktop-linux-amd64-3.3.1-linux1.deb -y 
```

If you have any question, [here](https://gist.github.com/berkorbay/6feda478a00b0432d13f1fc0a50467f1) is a link to follow. 

### Installing Vim on Ubuntu

Please run the following command in your terminal window: 

```bash
sudo apt install vim
```

If you have any question, [here](https://linuxhandbook.com/install-vim-ubuntu/) is a link to follow. 

### Installing VESC on Ubuntu

Step 1: Environment build-up. Please run the following command in your terminal window:

```bash
git clone https://github.com/vedderb/vesc_tool.git

sudo apt install qtbase5-dev qttools5-dev-tools qt5-default

sudo apt install qtquickcontrols2-5-dev

sudo apt-get install libqt5serialport5-dev

sudo apt-get install libqt5bluetooth5 qtpositioning5-dev libqt5gamepad5-dev

sudo apt-get install qtconnectivity5-dev
```

Step 2: Download the vesc_tool_free_linux.zip on website:

Link: https://vesc-project.com/vesc_tool

Step 3: Unzip the file in terminal window:

```bash
cd Downloads

unzip vesc_tool_free_linux.zip
```

If you have any question, [here](https://gist.github.com/takurx/941c80fce4619f65120b936dd719d0d7) is a link to follow.

### SSH key settings for the Github repository

Step 1: Log in to your own github account and go into the Settings: Access, and click the SSH and GPG keys.

Step 2: Click on the New SSH key to generate the key, you need to specify the Title, Key type and Key. It will automatically create a SSH key for you. Then click Add SSH key. Check also the link  https://docs.github.com/en/authentication/connecting-to-github-with-ssh

Step 3: Connect the repository to use SSH key. Go to the main page of your repository or the repository of Co-workers. Click on the Code, and go to the Local-Clone, then select SSH option and copy. Open your command terminal, then paste into the command line.

As an example, if the repository SSH option of our repository is is git@github.com:peter1zhang/F1_Tenth_Duke_Team.git, then in the command line, run:

```bash
git clone git@github.com:peter1zhang/F1_Tenth_Duke_Team.git
```

Step 4: It will automatically connect the repository with your SSH key and you are all done