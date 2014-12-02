package cz.vut.fit.xhajek31;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 *
 * @author damonkohler@google.com (Damon Kohler)
 */


public class ROS {

	String topicName;
	Publisher<std_msgs.String> publisher;

	public ROS (String name, ConnectedNode node)
	{
		topicName = name;
		publisher = node.newPublisher(topicName, std_msgs.String._TYPE);
	}

	public void send (int prob, String command)
	{
		std_msgs.String str = publisher.newMessage();
		str.setData(command);
		publisher.publish(str);
	}
}

