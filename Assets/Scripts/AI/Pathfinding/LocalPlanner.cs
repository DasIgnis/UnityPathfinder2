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
        public static List<PathNode> GetNeighbours(PathNode node, MovementProperties properties)
        {
            List<PathNode> result = new List<PathNode>();
            NavMeshHit currentArea;
            if (!NavMesh.SamplePosition(node.Position, out currentArea, 2f, NavMesh.AllAreas))
                return result;

            NavMeshHit area;
            PathNode next1 = new PathNode(
                new Vector3(node.Position.x + properties.maxSpeed, node.Position.y, node.Position.z + properties.maxSpeed));
            if (NavMesh.SamplePosition(next1.Position, out area, 2f, NavMesh.AllAreas) && currentArea.mask == area.mask)
                result.Add(next1);

            PathNode next2 = new PathNode(new Vector3(node.Position.x, node.Position.y, node.Position.z + properties.maxSpeed));
            if (NavMesh.SamplePosition(next2.Position, out area, 2f, NavMesh.AllAreas) && currentArea.mask == area.mask)
                result.Add(next2);

            PathNode next3 = new PathNode(new Vector3(node.Position.x + properties.maxSpeed, node.Position.y, node.Position.z));
            if (NavMesh.SamplePosition(next3.Position, out area, 2f, NavMesh.AllAreas) && currentArea.mask == area.mask)
                result.Add(next3);

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

            HashSet<PathNode> closed = new HashSet<PathNode>();
            Priority_Queue.SimplePriorityQueue<PathNode> opened = new Priority_Queue.SimplePriorityQueue<PathNode>();
            
            opened.Enqueue(position, 0);
            int steps = 0;

            PathNode last = opened.First;

            while (opened.Count != 0 && steps < 1000)
            {
                steps++;
                PathNode currentNode = opened.Dequeue();
                last = currentNode;
                closed.Add(currentNode);

                if (currentNode.EqualsSigma(target, movementProperties.epsilon)) break;

                //  Получаем список соседей
                var neighbours = GetNeighbours(currentNode, movementProperties);
                foreach (var nextNode in neighbours)
                {
                    float tentative = currentNode.G + currentNode.Distance(nextNode);

                    if (closed.Contains(nextNode) && tentative >= nextNode.G)
                    {
                        continue;
                    }

                    if ((!closed.Contains(nextNode) || tentative < nextNode.G))
                    {
                        nextNode.Parent = currentNode;
                        nextNode.G = tentative;
                        float heur = nextNode.G + currentNode.Distance(target);
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
