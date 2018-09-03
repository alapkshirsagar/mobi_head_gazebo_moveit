package com.tmericli.mobiheadros;

import org.apache.commons.logging.Log;
import org.ros.android.MessageCallable;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import std_msgs.String;

/**
 * This subscriber class is used for listening
 * the commands coming from the dashboard.
 *
 * @author Tekin Mericli
 * @version 1.0 2015
 */
public class MobiHeadROSSubscriberPublisher extends AbstractNodeMain {

    java.lang.String requestedExpression = "";

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("MobiHeadROSSubscriber");
    }
    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Log log = connectedNode.getLog();
        Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("/mobi_head_command", std_msgs.String._TYPE);
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                log.info("*********** I heard: \"" + message.getData() + "\"");
                requestedExpression = message.getData();
            }
        });
    }
}
