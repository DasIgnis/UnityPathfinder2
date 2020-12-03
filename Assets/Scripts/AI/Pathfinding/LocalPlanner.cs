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

        private static float Heur(PathNode node1, PathNode node2, MovementProperties properties)
        {
            //  Эвристику переделать - это пройденное время + оставшееся
            float angle = Mathf.Abs(Vector3.Angle(node1.Direction, node2.Position - node1.Position)) / properties.rotationAngle;
            //return node1.Distance(node2) + Mathf.Abs(angle);
            return node1.TimeMoment + 2*node1.Distance(node2)/properties.maxSpeed + angle*properties.deltaTime;
        }

        public static List<PathNode> GetNeighbours(PathNode node, MovementProperties properties)
        {
            //  Вот тут хардкодить не надо, это должно быть в properties
            //  У нас есть текущая точка, и свойства движения (там скорость, всякое такое)
            //float step = 1f;
            float step = properties.deltaTime * properties.maxSpeed;

            List<PathNode> result = new List<PathNode>();

            NavMeshHit currentArea;
            try
            {
                if (!NavMesh.SamplePosition(node.Position, out currentArea, 2f, NavMesh.AllAreas))
                    return result;
            }
            catch (Exception e)
            {
                Debug.Log("Shit happens : " + e.Message);
                return null;
            }

            for (int mult = 0; mult <= 1; ++mult)
                for (int angleStep = -properties.angleSteps; angleStep <= properties.angleSteps; ++angleStep)
                {
                    PathNode next = node.SpawnChildren(step * mult, angleStep * properties.rotationAngle, properties.deltaTime);
                    if (CheckWalkable(next.Position, currentArea.mask))
                    {
                        result.Add(next);
                        Debug.DrawLine(node.Position, next.Position, Color.white, 10f);
                    }
                }
            //Debug.Log("Children spawned : " + result.Count.ToString());

            return result;
        }

        public static List<PathNode> GetLocalRoute(
            PathNode target,
            PathNode position,
            MovementProperties movementProperties
            )
        {
            Debug.Log("Начато построение пути");

            //  Вот тут вместо equals надо использовать == (как минимум), а лучше измерять расстояние между точками
            //  сравнивая с некоторым epsilon. Хотя может это для каких-то специальных случаев?
            //  if (position.Position.Equals(target.Position)) return new List<PathNode>();
            if (Vector3.Distance(position.Position, target.Position) < movementProperties.epsilon) return new List<PathNode>();

            //HashSet<(Vector3, Vector3)> closed = new HashSet<(Vector3, Vector3)>();
            //HashSet<Vector3> closedDir = new HashSet<Vector3>();
            Priority_Queue.SimplePriorityQueue<PathNode> opened = new Priority_Queue.SimplePriorityQueue<PathNode>();

            //  Тут тоже вопрос - а почему с 0 добавляем? Хотя она же сразу извлекается, не важно
            opened.Enqueue(position, 0);
            int steps = 0;

            //  Посещенные узлы (с некоторым шагом, аналог сетки)
            HashSet<(int, int, int, int)> closed = new HashSet<(int, int, int, int)>();
            closed.Add(position.ToGrid4DPoint(movementProperties.deltaDist, movementProperties.deltaTime));

            PathNode last = opened.First;

            while (opened.Count != 0 && steps < 5000)
            {
                steps++;
                PathNode currentNode = opened.Dequeue();

                last = currentNode;
                //closed.Add(currentNode.ToGrid4DPoint(movementProperties.deltaDist, movementProperties.deltaTime));  //  Для Sample-based список closed не нужен

                //  Тут что-то более сложное
                if (currentNode.EqualsSigma(target, movementProperties.epsilon))
                {
                    Debug.Log("Braked by closest point. Steps : " + steps.ToString());
                    break;
                }

                //  Получаем список соседей
                var neighbours = GetNeighbours(currentNode, movementProperties);
                foreach (var nextNode in neighbours)
                {
                    var discreteNode = nextNode.ToGrid4DPoint(movementProperties.deltaDist, movementProperties.deltaTime);
                    if (!closed.Contains(discreteNode))
                    {
                        nextNode.H = Heur(nextNode, target, movementProperties);
                        opened.Enqueue(nextNode, nextNode.H);
                        closed.Add(discreteNode);
                    }
                }
            }
            
            if (last.EqualsSigma(target, movementProperties.epsilon) == false)
            {
                Debug.Log("Failed to build a way. Steps : " + steps.ToString());
                return new List<PathNode>();
            }

            List<PathNode> result = new List<PathNode>();

            //  Восстанавливаем путь от целевой к стартовой
            //  Может, заменить последнюю на целевую, с той же отметкой по времени? Но тогда с поворотом сложновато получается
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
