package edu.vutbr.xhajek31;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * Trida starajici se o publikovani vysledku
 */

public class ROS {

	private String topicName;
	private Publisher<std_msgs.String> publisher = null;
	private Publisher<geometry_msgs.Twist> publisherCmdVel = null;
	private final boolean executeCommand;
	private boolean paused = false;

	// /base_controller/command


	public ROS (String name, ConnectedNode node, boolean execute)
	{
		topicName = name;
		executeCommand = execute;
		if (!executeCommand) {
			publisher = node.newPublisher(topicName, std_msgs.String._TYPE);
		} else {
			publisherCmdVel = node.newPublisher("/base_controller/command", geometry_msgs.Twist._TYPE);
		}
		std_msgs.String str = publisher.newMessage();
		str.setData("starting");
		publisher.publish(str);
	}

	/**
	 * Slouzi k un/pausnuti zapisovani na topic
	 */

	public boolean togglePause ()
	{
		if (paused) {
			paused = false;
			RecognizerNode.log.info ("Publishing unpaused");
		} else {
			paused = true;
			RecognizerNode.log.info ("Publishing paused");
		}
		return paused;
	}

	/**
	 * Funkce zapise rozpoznany text na topic, resp. se pokusi vykonat prikaz (deprecated, nemusi fungovat)
	 */

	public void publish (String command, double prob)
	{
		String p = "[unpaused]";
		if (paused) {
			p = "[paused]";
		}
		RecognizerNode.log.info (p + " recognized \"" + command + "\" with probability " + prob);
		if (paused) {
			return;
		}
		if (executeCommand) {
			command = command.substring(12);		// remove "pee are two "
			geometry_msgs.Twist twist = publisherCmdVel.newMessage ();
			twist.getAngular().setZ(0);
      			twist.getLinear().setX(0);
			twist.getLinear().setY(0);
			if (command.equals ("go forward") || command.equals ("go front")) {
				twist.getLinear().setX(1);
			} else if (command.equals ("go backward") || command.equals ("go back")) {
				twist.getLinear().setX(-1);
			} else if (command.equals ("turn left") || command.equals ("rotate left")) {
				twist.getAngular().setZ(Math.PI / 2);
			} else if (command.equals ("turn right") || command.equals ("rotate right")) {
				twist.getAngular().setZ(-Math.PI / 2);
			} else if (command.equals ("stop")) {

			} else {
				return;
			}
			publisherCmdVel.publish (twist);
		} else {
			std_msgs.String str = publisher.newMessage();
			str.setData(command);
			publisher.publish(str);
		}
	}
}
