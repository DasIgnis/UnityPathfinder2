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
    public static class GlobalPlanner
    {

        public static Dictionary<Region, Dictionary<Region, float>> InitDijkstraVertices(List<Region> regions)
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

        public static List<Region> GetPathWithDijkstra(Region start, Region target, List<Region> points)
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
            return path;
        }

        public static List<Region> GetNextPoint(Region start, Region target, List<Region> points)
        {

            List<Region> res = new List<Region>();
            Dictionary<Region, int> marks = new Dictionary<Region, int>();
            int markVal = 0;
            marks.Add(start, markVal);

            while (!(marks.ContainsKey(target)) && marks.Keys.ToList() != points) //пока не пометили таргет и пока не пометили вообще всё
            {
                var markedRegions = marks.Where(el => el.Value == markVal).ToList().Select(x => x.Key); // берем точки, помеченные числом d

                foreach (var reg in markedRegions)
                {
                    Cartographer cartographer = new Cartographer();
                    var neighbours = cartographer.GetNeighbours(reg.Index, reg.PathPoints.First()); //по какой точке брать соседей?

                    foreach (var unmarked in neighbours)
                        marks.Add(unmarked, markVal + 1); //помечаем все эти точки числом d+1
                }
                markVal += 1;
            }

            if (marks.ContainsKey(target))
            {
                var current = target;

                while (current != start)
                {
                    Cartographer cartographer = new Cartographer();
                    var neighbours = cartographer.GetNeighbours(current.Index, current.PathPoints.First());

                    var next = marks.First(p => neighbours.Contains(p.Key) && marks[p.Key] == (marks[current] - 1)).Key; //
                    res.Add(next);
                    current = next;
                }
            }
            return res; //возвращаем путь или пустой список
        }

        public static PathNode GetGlobalRoute(PathNode target, PathNode position, MovementProperties movementProperties)
        {
            NavMeshHit targetArea;
            if (!NavMesh.SamplePosition(target.Position, out targetArea, 1f, NavMesh.AllAreas))
                return null;

            Vector3 adjustedPosition = new Vector3(position.Position.x, targetArea.position.y, position.Position.z);
            if (target.Distance(adjustedPosition) < movementProperties.epsilon)
                return null;

            NavMeshHit currentArea;
            if (!NavMesh.SamplePosition(adjustedPosition, out currentArea, 1f, NavMesh.AllAreas))
                return null;

            if (currentArea.mask == targetArea.mask)
            {
                target.RegionId = currentArea.mask;
                return target;
            }

            //TODO: Здесь надо реализовать простой волновой алгоритм, согласно которому мы выбираем следующую точку
            //В результате работы волнового алгоритма мы получим список регионов, по которым мы должны пройти
            //Берём следующий регион, куда мы должны прийти

            Cartographer cartographer = new Cartographer();
            var neighbours = cartographer.GetNeighbours(currentArea.mask, adjustedPosition);

            neighbours.Reverse();

            if (neighbours.Count > 1 && neighbours[0].PathPoints.Count > 0)
            {
                var nextRegion = neighbours[0];
                //TODO: 
                //Этот код для региона, куда мы должны пойти, определяет дальнейшую точку маршрута планировщика
                //Здесь везде вместо neighbours[0] должен быть регион, куда мы идем
                //Для простых регионов мы просто определяем, в какую сторону должен воевать локальный планировщик
                if (nextRegion.Type == RegionType.Stable)
                {
                    NavMeshHit moveTo;
                    //Ищем ближайшую точку текущего региона, соседствующаю с целевым регионом
                    if (NavMesh.SamplePosition(nextRegion.PathPoints[0], out moveTo, 20, currentArea.mask))
                    {
                        //Если мы в данный момент находимся близко к найденной точке, то мы на границе
                        //В этом случае возвращаем точку из соседнего региона
                        if (Math.Abs(moveTo.position.x - adjustedPosition.x) < movementProperties.deltaTime * movementProperties.maxSpeed
                            && Math.Abs(moveTo.position.y - adjustedPosition.y) < movementProperties.deltaTime * movementProperties.maxSpeed)
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
                }
                else if (nextRegion.Type == RegionType.Moving)
                {
                    PathNode regionExit = null;
                    foreach (var exit in ((MovingRegion)nextRegion).ExitPoints)
                    {

                    }
                }

                return new PathNode(neighbours[0].PathPoints[0]);
            }

            return null;
        }
    }
}
