using Assets.Scripts.AI.Extensions;
using BaseAI;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.AI;

namespace Assets.Scripts.AI.Pathfinding
{
    public static class LocalPlanner
    {
        private static bool CheckWalkable(Vector3 point, int areaId)
        {
            NavMeshHit area;
            NavMesh.SamplePosition(point, out area, 2f, NavMesh.AllAreas);
            return area.mask == areaId;
        }

        private static float Heur(PathNode node1, PathNode node2)
        {
            float angle = Vector3.Angle(node1.Direction, node2.Position - node1.Position) * Mathf.Deg2Rad;
            return node1.Distance(node2) + Mathf.Abs(angle);
        }

        public static List<PathNode> GetNeighbours(PathNode node, MovementProperties properties)
        {
            float step = 5f;
            float angle = 1f;
            List<PathNode> result = new List<PathNode>();
            NavMeshHit currentArea;
            if (!NavMesh.SamplePosition(node.Position, out currentArea, 2f, NavMesh.AllAreas))
                return result;

            Vector3 parentPos = node.Position;

            PathNode next1 = new PathNode(node.Position + node.Direction.normalized * step);
            if (CheckWalkable(next1.Position, currentArea.mask))
                result.Add(next1);

            
            PathNode next2 = new PathNode(node.Position);
            next2.Direction = Quaternion.Euler(0, Mathf.Rad2Deg * angle, 0) * next2.Direction;
            //if (CheckWalkable(next2.Position, currentArea.mask))
            //    result.Add(next2);

            PathNode next3 = new PathNode(node.Position);
            next3.Direction = Quaternion.Euler(0, Mathf.Rad2Deg * -angle, 0) * next3.Direction;
            //if (CheckWalkable(next3.Position, currentArea.mask))
            //    result.Add(next3);

            PathNode next4 = new PathNode(node.Position + next2.Direction * step);
            next4.Direction = next2.Direction;
            if (CheckWalkable(next4.Position, currentArea.mask))
                result.Add(next4);
            if (!CheckWalkable(next4.Position, currentArea.mask))
                Debug.Log("not walkable");

            PathNode next5 = new PathNode(node.Position + next3.Direction * step);
            next5.Direction = next3.Direction;
            if (CheckWalkable(next5.Position, currentArea.mask))
                result.Add(next5);
            if (!CheckWalkable(next5.Position, currentArea.mask))
                Debug.Log("not walkable");

            return result;
        }

        public static List<PathNode> GetLocalRoute(
            PathNode target, 
            PathNode position, 
            MovementProperties movementProperties
            )
        {
            if (position.Position.Equals(target.Position))
                return new List<PathNode>();

            HashSet<(Vector3, Vector3)> closed = new HashSet<(Vector3, Vector3)>();
            HashSet<Vector3> closedDir = new HashSet<Vector3>();
            Priority_Queue.SimplePriorityQueue<PathNode> opened = new Priority_Queue.SimplePriorityQueue<PathNode>();
            
            opened.Enqueue(position, 0);
            int steps = 0;

            PathNode last = opened.First;

            while (opened.Count != 0 && steps < 10000)
            {
                steps++;
                PathNode currentNode = opened.Dequeue();
                last = currentNode;
                closed.Add((currentNode.Position, currentNode.Direction));

                if (currentNode.EqualsSigma(target, movementProperties.maxSpeed)) 
                    break;

                //  Получаем список соседей
                var neighbours = GetNeighbours(currentNode, movementProperties);
                foreach (var nextNode in neighbours)
                {
                    float tentative = currentNode.G + Heur(currentNode, nextNode);

                    if (closed.Contains((nextNode.Position, nextNode.Direction)) && tentative >= nextNode.G)
                    {
                        continue;
                    }

                    if ((!closed.Contains((nextNode.Position, nextNode.Direction)) || tentative < nextNode.G))
                    {
                        nextNode.Parent = currentNode;
                        nextNode.G = tentative;
                        float heur = nextNode.G
                            + Heur(nextNode, target);
                        if (opened.Contains(nextNode))
                            opened.Remove(nextNode);
                        opened.Enqueue(nextNode, heur);
                    }
                }
            }

            List<PathNode> result = new List<PathNode>();

            //  Восстанавливаем путь от целевой к стартовой
            var pathElem = last;
            while (pathElem != null)
            {
                result.Add(pathElem);
                pathElem = pathElem.Parent;
            }

            result.Reverse();
            result.RemoveAt(0);

            return result;
        }
    }
}
