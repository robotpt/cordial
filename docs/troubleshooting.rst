Troubleshooting
===============

Unable to contact ROS master at 192.168.100.1:11311
---------------------------------------------------

This means that the two computers in QT are not talking correctly because they did not boot in the right order.
To fix this, restart QT by pressing down QT's power button for a second or two.
QT should slump forward and the face screen should go off.
If the face screen doesn't go off, power QT on and repeat the process.
I haven't ever had to do this more than twice, as by the second time they are synced.

To test if the computers are talking correctly, you should be enter the following command and see a list of outputs.  If you get an error, restart QT.

::

    rostopic list

The tablet doesn't connect
--------------------------

This can be two things in my experience:

1. The tablet is on the wrong wifi network
2. The interaction isn't running (most commonly Fitbit credential errors)


Tablet is connecting to the wrong wifi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is easiest to check that the tablet is on the correct wifi.
When QT turns off, it turns off the wireless network that it is hosting.
When this happens, most tablets will auto-join other networks that they have been connected to.
What makes this worse is that web browsers often rememeber (cache) websites to make them quicker to load, which makes it look like you're connecting to the website hosted by QT, but you're not.
To fix this, go into the tablet's settings and turn off autojoin for all wireless networks except for QT (or just the networks you expect the tablet to see while the interaction is running).
Once this is set, you should be able to connect to and prompt the interaction on QT, including after QT restarts (if you've changed the container's restart policy).


QT's head computer doesn't have internet
----------------------------------------

If you can SSH into QT's head computer, but are unable to clone a repository or make an update, check if you have internet.  The following command should show you that messages are being sent and responses are received, it just hangs there, you are not connected to the internet::

    ping google.com

At the moment, just restart QT and hopefully the problem will be fixed.
LuxAI has recognized this problem and given me steps to fix it but I haven't tested them.

If you would like to try, here is their email

    In our older setup of RPI/NUC network (same as your QT) we had enabled port forwarding (8080 -> 80) on both machine to facilitate accessing the QTrobot wev config. In some cases and time-to-time (also depending on the router) this causes problem for RPI to reach the internet properly.


    In this FAQ we explained it how to disable it: https://docs.luxai.com/FAQ/Network/

    Some more comments on this issue : https://github.com/luxai-qtrobot/QA/issues/3


    However all you need to do (as I imagine this can be the cause of the problem) is to disable the port forwarding on both machines QTPC and QTRP.

    For QTRP (RPI), just follow the simple instruction in the FAQ link and comment the corresponding line in :code`start_qt_routes.sh`.
    Double check that your :code:`/etc/network/interfaces` on RPI has the following config::

        auto lo eth0
        iface lo inet loopback
        auto eth0
        iface eth0 inet static
        address 192.168.100.1
        netmask 255.255.255.0
        gateway 192.168.100.2
        dns-nameservers 192.168.100.2 8.8.8.8

    Regarding the QTPC, you can completely disable/remove the ‘start_qt_routes.sh’.
