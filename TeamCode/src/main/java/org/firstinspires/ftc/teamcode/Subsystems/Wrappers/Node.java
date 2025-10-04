package org.firstinspires.ftc.teamcode.Subsystems.Wrappers;

///  - function that moves to a certain point on the graph
///  - FUnction that scheudles actions at certain points
///  - Get robot position
/// Stop
public class Node {
    //       inch, inch, degrees
    public double X, Y, Theta;
    public Node Previous;
    public Node Next;
    public Node() {}
    public Node(double x, double y, double theta) {
        Theta = theta;
        X = x;
        Y = y;
    }
    public Node nextNode(double x, double y, double theta) {
        Theta = theta;
        X = x;
        Y = y;
        Node n = new Node();
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
