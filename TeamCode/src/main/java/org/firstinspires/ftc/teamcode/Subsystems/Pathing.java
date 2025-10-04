package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Subsystems.Wrappers.Node;

public class Pathing {

    public Node createPath(Node node) {
        while (node.Previous != null) {
            node = node.Previous;
        }
        return node;
    }

    public Pathing (){

    }

}