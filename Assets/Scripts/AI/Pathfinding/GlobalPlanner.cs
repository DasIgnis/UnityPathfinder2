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
    public class GlobalPlanner
    {
        private List<Region> globalPath = new List<Region>();

        public Dictionary<Region, Dictionary<Region, float>> InitDijkstraVertices(List<Region> regions)
        {
            var res = new Dictionary<Region, Dictionary<Region, float>>();

            Cartographer cartographer = new Cartographer();

            foreach (Region reg in regions)
            {
                var neighbours = cartographer.GetNeighbours(reg.Index, reg.PathPoints.First());
                var neighboursDict = new Dictionary<Region, float>();
                foreach (var n in neighbours)
                {
                    neighboursDict.Add(n, n.Price);
                    res[reg] = neighboursDict;
                }
            }
            return res;
        }

        public List<Region> GetPathWithDijkstra(Region start, Region target, List<Region> points)
        {
            var prevRegions = new Dictionary<Region, Region>();
            var distances = new Dictionary<Region, float>();
            var nodes = new List<Region>();

            List<Region> path = null;

            var vertices = InitDijkstraVertices(points);

            foreach (var reg in vertices)
            {
                distances[reg.Key] = reg.Key == start ? 0 : int.MaxValue;
                nodes.Add(reg.Key);
            }

            while (nodes.Count != 0)
            {
                nodes.Sort((x, y) => (int)(distances[x] - distances[y])); //опасное приведение типов?

                var min = nodes[0];
                nodes.Remove(min);

                if (min == target)
                {
                    path = new List<Region>();
                    while (prevRegions.ContainsKey(min))
                    {
                        path.Add(min);
                        min = prevRegions[min];
                    }
                    break;
                }

                if (distances[min] == int.MaxValue)
                    break;

                foreach (var neighbour in vertices[min])
                {
                    var alt = distances[min] + neighbour.Value;
                    if (alt < distances[neighbour.Key])
                    {
                        distances[neighbour.Key] = alt;
                        prevRegions[neighbour.Key] = min;
                    }
                }
            }
            path.Reverse();
            return path;
        }

        //public static List<Region> GetNextPoint(Region start, Region target, List<Region> points)
        //{

        //    List<Region> res = new List<Region>();
        //    Dictionary<Region, int> marks = new Dictionary<Region, int>();
        //    int markVal = 0;
        //    marks.Add(start, markVal);

        //    while (!(marks.ContainsKey(target)) && marks.Keys.ToList() != points) //пока не пометили таргет и пока не пометили вообще всё
        //    {
        //        var markedRegions = marks.Where(el => el.Value == markVal).ToList().Select(x => x.Key); // берем точки, помеченные числом d

        //        foreach (var reg in markedRegions)
        //        {
        //            Cartographer cartographer = new Cartographer();
        //            var neighbours = cartographer.GetNeighbours(reg.Index, reg.PathPoints.First()); //по какой точке брать соседей?

        //            foreach (var unmarked in neighbours)
        //                marks.Add(unmarked, markVal + 1); //помечаем все эти точки числом d+1
        //        }
        //        markVal += 1;
        //    }

        //    if (marks.ContainsKey(target))
        //    {
        //        var current = target;

        //        while (current != start)
        //        {
        //            Cartographer cartographer = new Cartographer();
        //            var neighbours = cartographer.GetNeighbours(current.Index, current.PathPoints.First());

        //            var next = marks.First(p => neighbours.Contains(p.Key) && marks[p.Key] == (marks[current] - 1)).Key; //
        //            res.Add(next);
        //            current = next;
        //        }
        //    }
        //    return res; //возвращаем путь или пустой список
        //}

        public PathNode GetGlobalRoute(PathNode target, PathNode position, MovementProperties movementProperties)
        {
            NavMeshHit targetArea;
            if (!NavMesh.SamplePosition(target.Position, out targetArea, 1f, NavMesh.AllAreas))
                return null;

            Vector3 adjustedPosition = new Vector3(position.Position.x, targetArea.position.y, position.Position.z);
            if (target.Distance(adjustedPosition) < movementProperties.epsilon)
                return null;

            NavMeshHit currentArea;
            if (!NavMesh.SamplePosition(adjustedPosition, out currentArea, 1f, NavMesh.AllAreas)
                && !(globalPath.Count == 0)
                && !(globalPath.First().Type == RegionType.Moving))
                return null;

            if (NavMesh.SamplePosition(adjustedPosition, out _, 1f, NavMesh.AllAreas) 
                && currentArea.mask == targetArea.mask)
            {
                if (targetArea.position.x - currentArea.position.x < movementProperties.deltaTime * movementProperties.maxSpeed
                    && targetArea.position.z - currentArea.position.z < movementProperties.deltaTime * movementProperties.maxSpeed)
                {
                    return null;
                }

                globalPath = new List<Region>();
                target.RegionId = currentArea.mask;
                return target;
            }

            Cartographer cartographer = new Cartographer();

            if (globalPath.Count == 0)
            {
                globalPath = new List<Region>();
            }

            if (globalPath.Count != 0)
            {
                var nextRegion = globalPath.First();

                if (currentArea.mask == nextRegion.Index)
                {
                    globalPath.RemoveAt(0);
                    nextRegion = globalPath.First();
                }

                if (nextRegion.Type == RegionType.Stable)
                {
                    return calcStableRegionPath(nextRegion, currentArea, adjustedPosition, movementProperties);
                }
                else if (nextRegion.Type == RegionType.Moving)
                {
                    var movingRegion = (MovingRegion)nextRegion;

                    foreach (var exit in movingRegion.ExitPoints)
                    {
                        //ищем точку прыжка на платформу в текущей области
                        NavMeshHit exitHit;
                        if (NavMesh.SamplePosition(exit.transform.position, out exitHit, 20, currentArea.mask))
                        {
                            if (exitHit.mask == currentArea.mask)
                            {
                                if (Math.Abs(exitHit.position.x - adjustedPosition.x) < movementProperties.deltaTime * movementProperties.maxSpeed
                                    && Math.Abs(exitHit.position.y - adjustedPosition.y) < movementProperties.deltaTime * movementProperties.maxSpeed)
                                {
                                    //Если мы в ней, то ждем платформу 
                                    var node = new PathNode(movingRegion.EntryPoint.transform.position);
                                    node.RegionId = nextRegion.Index;
                                    node.JumpingPosition = true;
                                    return node;
                                }
                                else
                                {
                                    //Если не в ней, то топаем к ней
                                    var node = new PathNode(exitHit.position);
                                    node.RegionId = currentArea.mask;
                                    return node;
                                }
                            }
                        }
                    }

                    //Если мы уже на платформе, то идем следующий регион среди выходных точек и ждем, пока доедем
                    //Либо если мы уже на входе в следующий регион, то топаем дальше
                    var entryPointPosition = movingRegion.EntryPoint.transform.position;
                    if (entryPointPosition.x - adjustedPosition.x < 1f
                        && entryPointPosition.z - adjustedPosition.z < 1f)
                    {
                        if (globalPath.Count < 2)
                        {
                            //Мы уже у цели
                            globalPath = new List<Region>();
                            return null;
                        }

                        var exitRegion = globalPath[1];
                        foreach (var exit in movingRegion.ExitPoints)
                        {
                            NavMeshHit exitHit;
                            if (NavMesh.SamplePosition(exit.transform.position, out exitHit, 20, exitRegion.Index))
                            {
                                //Мы нашли точку выхода в следующую зону
                                //Если мы в ней, то мы успешно прошли платформу
                                if (Math.Abs(exitHit.position.x - adjustedPosition.x) < movementProperties.deltaTime * movementProperties.maxSpeed
                                    && Math.Abs(exitHit.position.z - adjustedPosition.z) < movementProperties.deltaTime * movementProperties.maxSpeed)
                                {
                                    globalPath.RemoveAt(0);
                                    var node = new PathNode(exitHit.position);
                                    node.RegionId = globalPath.First().Index;
                                    return node;
                                }
                                //Иначе ждем приближения точки
                                else
                                {
                                    var node = new PathNode(exit.transform.position);
                                    node.RegionId = nextRegion.Index;
                                    node.JumpingPosition = true;
                                    return node;
                                }
                            }
                        }
                    }
                }

                return new PathNode(nextRegion.PathPoints[0]);
            }

            return null;
        }

        private PathNode calcStableRegionPath(Region nextRegion, NavMeshHit currentArea, Vector3 adjustedPosition, MovementProperties movementProperties)
        {
            NavMeshHit moveTo;
            //Ищем ближайшую точку текущего региона, соседствующаю с целевым регионом
            if (NavMesh.SamplePosition(nextRegion.PathPoints[0], out moveTo, 20, currentArea.mask))
            {
                //Если мы в данный момент находимся близко к найденной точке, то мы на границе
                //В этом случае возвращаем точку из соседнего региона
                if (Math.Abs(moveTo.position.x - adjustedPosition.x) < movementProperties.deltaTime * movementProperties.maxSpeed
                    && Math.Abs(moveTo.position.z - adjustedPosition.z) < movementProperties.deltaTime * movementProperties.maxSpeed)
                {
                    var node = new PathNode(nextRegion.PathPoints[0]);
                    node.RegionId = nextRegion.Index;
                    return node;
                }
                //Иначе мы возвращаем ближайшую к цели точку из текущего региона, чтобы скормить локальному планировщику
                else
                {
                    var node = new PathNode(moveTo.position);
                    node.RegionId = currentArea.mask;
                    return node;
                }
            };
            return null;
        }
    }
}
