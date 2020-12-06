﻿using BaseAI;
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

        public List<PathNode> GetNextPoint(Region start, Region target, List<Region> points)
        {

            List<PathNode> res = new List<PathNode>();
            Dictionary<Region, int> marks = new Dictionary<Region, int>();
            int markVal = 0;
            marks.Add(start, markVal);

            while (!(marks.ContainsKey(target)) && marks.Keys.ToList() != points) //пока не пометили таргет и пока не пометили вообще всё
             {
                var markedRegions = marks.Where(el => el.Value == 0).ToList().Select(x => x.Key); // берем точки, помеченные числом d

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

                    //var next = marks.First(p => neighbours.Contains(p) && marks[p] == (marks[current] - 1)); //
                    //res.Add(next);
                    //current = next;
                }
            }
            return res; //возвращаем путь или пустой список
        }

        public PathNode GetGlobalRoute(PathNode target, PathNode position, MovementProperties movementProperties)
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

            Cartographer cartographer = new Cartographer();

            if (globalPath.Count == 0)
            {
                globalPath.AddRange(cartographer.GetNeighbours(8, position.Position));
                globalPath.AddRange(cartographer.GetNeighbours(128, globalPath[0].PathPoints[0]));
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

                return new PathNode(nextRegion.PathPoints[0]);
            }

            return null;
        }
    }
}
