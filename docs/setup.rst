Setting up QT
=============

.. |version| replace:: master

What you'll need
----------------

* A monitor
* A mouse and keyboard
* Wireless internet
* An `Amazon Web Services <https://aws.amazon.com/>`_ account
* A Google account to get an app for the tablet

QT has two ports: one USB-C and one USB-A.  You'll need to use the USB-C port for display and figure out how to control a mouse and keyboard.  I suggest a USB-C hub that has a display port that you can use (USB-C or HDMI, for example) and has USB-A ports for your mouse and keyboard.  Alternatively, you can use a USB-A hub to connect your mouse and keyboard to QT.

.. note::

    If you have trouble using your mouse or keyboard through a USB-C port, try flipping the USB-C input going into QT.  In theory, USB-C should go both ways, but in practice, sometimes not.

Basics
------

Turning QT on and off
^^^^^^^^^^^^^^^^^^^^^

To turn on QT, just plug in power to QT and it will boot.

To turn off QT, there are two options:

    1. Press the button on the backside of QT, near its feet.

    2. Login to the head computer (see below) and do :code:`sudo shutdown`.

If you do :code:`sudo shutdown` on the body computer, you only turn off the body computer---the head computer is still on.

.. note::

    If you unplug QT to restart QT, it may mess up the boot timing of the two computers.  Probably one of them takes longer because it boots in recovery mode.  This screws up how the head and body computer network.  You will not be able to connect to the head computer from the body computer.  If this occurs, simple restart QT by pushing the button on its backside.

Accessing QT's body computer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When you connect a monitor to QT and turn QT on, you will start on QT's body computer.

Accessing QT's head computer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To setup the head, you must Secure-SHell into it (SSH) from QT's body computer.  To do this

0. Turn on QT.

1. Open a terminal.

2. Type the following and hit return::

    ssh qtrobot@192.168.100.1

Head
----

Turning off the default face
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0. If you haven't already, SSH into QT's head computer::

    ssh qtrobot@192.168.100.1

1. Update QT::

    cd ~/robot/packages/deb
    git pull
    sudo dpkg -i ros-kinetic-qt-robot-interface_1.1.8-0xenial_armhf.deb

.. note::

    If the :code:`git pull` step fails, the head computer might be having trouble with it its network.  You can check this with :code:`ping google.com`.  If there's nothing, there is a problem with the network.  To fix this, the best think we've found is to restart QT: :code:`sudo reboot`.

2. Edit a configuration file to turn off QT's default face:

    a. Open the configuration file::

        sudo nano /opt/ros/kinetic/share/qt_robot_interface/config/qtrobot-interface.yaml

    b. Change the line that says :code:`disable_interface: false` to :code:`disable_interface: true`

    c. Save and exit :code:`nano` by hitting Ctrl+x, then typing 'y', and then hitting Enter twice to confirm things.

.. note::

    You can reboot to see these changes take effect, or continue on and we'll reboot eventually.

Setting up our code
^^^^^^^^^^^^^^^^^^^

0. Secure-Shell (SSH) into QT's head computer::

    ssh qtrobot@192.168.100.1

1. Install our project's dependencies:

    .. parsed-literal::

        git clone -b |version| https://github.com/robotpt/cordial-docker.git
        bash ~/cordial-docker/scripts/pi_setup.bash

2. Increase the swap size, so we're able to build without running out of virtual memory:

    a. Turn off your swap memory::

        sudo /sbin/dphys-swapfile swapoff

    b. Open your swap configuration file::

        sudo nano /etc/dphys-swapfile

    c. Set `CONF_SWAPFACTOR` to 2 by changing the line that says :code:`#CONF_SWAPFACTOR=2` to :code:`CONF_SWAPFACTOR=2`, that is by deleting the :code:`#` character to uncomment the line. 

    d. Save and exit :code:`nano` by hitting Ctrl+x, then typing 'y', and then hitting Enter twice to confirm things.

    e. Turn the swap file back on::

        sudo /sbin/dphys-swapfile swapon

3. Clone our repositories and build them:

    a. Go to the source code directory in the catkin workspace::

        cd ~/catkin_ws/src

    b. Clone our repositories:

        .. parsed-literal::

            git clone -b |version| https://github.com/robotpt/cordial
            git clone -b |version| https://github.com/robotpt/qt-robot

    c. Build our workspace::

        cd ~/catkin_ws
        catkin_make

    .. note::

        It takes around five minutes for this command to finish.  You can setup QT's body computer at the same time as it runs, if you like.

4. Setup our code to run when QT's head computer turns on.

    a. Copy the autostart script into the correct directory::

        roscp qt_robot_pi start_usc.sh /home/qtrobot/robot/autostart/

    b. Enable the autostart script:

        i. Open a webbrowser on QT (e.g., Firefox) and go to `http://192.168.100.1:8080/ <http://192.168.100.1:8080/>`_.

        .. figure:: images/qt_menu.png
            :align: center

            QT's configuration menu.

        ii. Click 'Autostart'.  You'll be prompted for a username and password. Enter :code:`qtrobot` for both.

        iii. Click the 'Active' checkbox next to :code:`start_usc.sh`.

        .. figure:: images/autostart_checked.png
            :align: center

            QT's autostart menu with our script, :code:`start_usc.sh`, checked.

        iv. Click 'Save' and then 'Return' twice.

.. note::

    You can reboot to see these changes take effect, or continue on and we'll reboot eventually.

    If you'd like, you can confirm that things are running after a reboot by opening a terminal and running the following command.  You should see both :code:`/sound_listener` and :code:`/start_face_server`::

       rosnode list | grep "/\(sound_listener\|start_face_server\)"

    .. figure:: images/head_nodes_running.png
        :align: center

        What you should see if the head nodes are running correctly.

Body
----

Getting your Amazon Web Service credentials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For QT to speak, we use Amazon Polly, which requires an Amazon Web Services account. At our current usage, using `Amazon Polly is free up to a certain level <https://aws.amazon.com/polly/pricing/>`_), but you will need a credit card to create an account.

1. `Create an Amazon Web Services account <https://portal.aws.amazon.com/billing/signup#/start>`_.
2. Once you sign in, in the top right of the page, click your account name (mine says "Audrow"), then in the drop-down menu click "My Security Credentials," then click "Create New Access Key."
3. Record your access key and keep it somewhere safe.  You can do this by downloading this or just viewing it and copy-pasting it to somewhere for later reference.

.. note::

    It is best practice to create separate accounts with less access than your root account and use those access keys, see `Amazon's security best practices <https://aws.amazon.com/blogs/security/getting-started-follow-security-best-practices-as-you-configure-your-aws-resources/>`_.


Setting up our interaction in a Docker container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0. Change your system timezone to be in your current timezone.  To do this, you can click the time in the upper-right of the desktop on QT and then click 'Time & Date settings...'

1. Open a terminal and clone this repository onto QT's body computer:

    .. parsed-literal::

        git clone -b |version| https://github.com/robotpt/cordial-docker ~/cordial-docker

2. Run a script to allow for updates::

    sudo bash ~/cordial-docker/scripts/nuc_setup.bash

.. warning::

    If this step fails, try the following commands before rerunning::

        sudo apt install --reinstall python3-six
        sudo apt install --reinstall python3-chardet

.. note::

    This step takes five minutes or so.

3. Setup Docker:

    a. Install Docker::

        curl -fsSL https://get.docker.com -o get-docker.sh
        sh get-docker.sh

    b. Set Docker to run without :code:`sudo`::

        sudo groupadd docker
        sudo gpasswd -a $USER docker
        newgrp docker

    c. Test that Docker is installed correctly and works without :code:`sudo`::

        docker run hello-world

    .. figure:: images/hello_from_docker.png
        :align: center

        What is printed from running the :code:`hello-world` docker container.


4. Setup Docker-compose:

    a. Install Docker-compose::

        sudo curl -L "https://github.com/docker/compose/releases/download/1.25.3/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
        sudo chmod +x /usr/local/bin/docker-compose

    b. Check that docker compose is installed correctly::

        docker-compose version


5. Setup the docker container::

    bash ~/cordial-docker/docker/run.sh

    .. note::

        The first time that you run the Docker script, it will take around 15 minutes (depending on your internet) to setup the container.  After that, it will be fast.  Feel free to take a break or go get coffee :-)

    .. note::

        I did have an error occur during this command one of the times I was setting it up.  It might have been a network issue.  I ran it again and it succeeded.  If you have trouble here let me know.

6. Configure your AWS credentials by typing :code:`aws configure` in the terminal that pops up.


Setting up remote access to QT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Get `Dataplicity <https://www.dataplicity.com/devices/>`_ login credentials from Audrow and sign on.  Go to the devices tab and then click "+ Add New Device".  Copy or enter this command into a terminal on QT's body PC and enter QT's password 'qtrobot'.  After that runs, remote access should be setup.  You can confirm this by clicking the added device and confirming that you can explore the file system (e.g., :code:`ls /home/qtrobot` and you should see familiar directories such as :code:`cordial-docker`).
