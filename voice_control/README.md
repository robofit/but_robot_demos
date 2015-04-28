voice_control
=============

Instalation
-----------
``
catkin_make install
source install/setup.bash
``

Run
---

Recognition tool:

* sphinx-4 with JSGF grammar: `roslaunch voice_control voice_control.launch`
* sphinx-4 with JSGF grammar and command execution: `roslaunch voice_control voice_control_execute.launch`
* sphinx-4 with language model: `roslaunch voice_control voice_control_lm.launch`
* GoogleSpeechAPI: `roslaunch voice_control voice_control_google.launch`

JSGF grammar files can be set in `voice_control.launch` file.
For using GSpeechAPI, fill in key to `voice_control.launch` file.

Other settings, like accoustic model, can be changed in proper sphinx-4 config file (`data/config/default_config.xml`).

When using sphinx-4 library, I strongly recommend pausing output (p + Enter) to calibrate library - say a few phrases and check whether app prints correct results. To continue publishing, press p + Enter again. Calibration can be done at any time.

To execute example python script, run: `roslaunch voice_control_examples examples.launch`.
It expects phrases from predefined grammar.
