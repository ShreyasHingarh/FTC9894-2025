package org.firstinspires.ftc.teamcode.Subsystems.Wrappers;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
///  - FUnction that scheudles actions at certain points
/// Stop

public class Node {
    public final class ActionScheduler {
        private final Action action;
        private final double percentToStart;

        public ActionScheduler(Action action, double percentToStart) {
            this.action = action;
            this.percentToStart = percentToStart;
        }
        public Action getAction() {
            return action;
        }
        public double getPercentToStart() {
            return percentToStart;
        }
        @Override
        public String toString() {
            return "ActionScheduler[" +
                    "action=" + action + ", " +
                    "percentToStart=" + percentToStart + ']';
        }
    }
    //       inch, inch, degrees
    public double X, Y, Theta, milliseconds;
    public Node Previous;
    public ActionScheduler[] actionSchedulers;
    public Node Next;
    public Node() {}
    public Node(double x, double y, double theta) {
        Theta = theta;
        X = x;
        Y = y;
    }
    public Node nextPositionNode(double x, double y, double theta) {
        Theta = theta;
        X = x;
        Y = y;
        Node n = new Node();
        Next = n;
        n.Previous = this;
        return n;
    }
    public Node nextPositionNode(double x, double y, double theta, ActionScheduler... movementToStartAt) {
        Theta = theta;
        X = x;
        Y = y;
        milliseconds = -1;
        Node n = new Node();
        Next = n;
        actionSchedulers = movementToStartAt;
        n.Previous = this;
        return n;
    }
    public Node pause(double ms){
        Node n = new Node();
        milliseconds = ms;
        Next = n;
        n.Previous = this;
        return n;
    }
    public Node end() {
        Next = new Node();
        Next.Previous = this;
        return Next;
    }
}
