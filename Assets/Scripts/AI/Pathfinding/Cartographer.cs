using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.AI;

namespace Assets.Scripts.AI.Pathfinding
{
    public enum RegionType
    {
        Stable,
        Moving
    }

    public class Region
    {
        public List<Vector3> PathPoints { get; set; }
        public int Index { get; set; }
        public RegionType Type { get; set; }
        public float Price { get; set; }
        public Region()
        {
            PathPoints = new List<Vector3>();
        }
    }

    public class MovingRegion: Region
    {
        public GameObject EntryPoint { get; set; }
        public List<GameObject> ExitPoints { get; set; }
    }

    //Можно сделать статическим, но слишком много мороки с инициализациями переменных
    public class Cartographer
    {
        private Dictionary<int, List<int>> neigbours = new Dictionary<int, List<int>>();
        private List<Region> regions = new List<Region>();
        public Cartographer()
        {
            regions.Add(new Region { Index = 8, Type = RegionType.Stable });
            regions.Add(new MovingRegion { 
                Index = 64, 
                Type = RegionType.Moving, 
                EntryPoint = GameObject.Find("Platform1Entry"),
                ExitPoints = new List<GameObject> { GameObject.Find("Platform1TerrainExit1"), GameObject.Find("Platform1TerrainExit2") }
            });;
            regions.Add(new Region { Index = 128, Type = RegionType.Stable });

            neigbours.Add(8, new List<int> { 64, 128 });
        }

        public List<Region> GetNeighbours(int regionIndex, Vector3 inregionPosition)
        {
            List<Region> result = regions.Where(x => neigbours[regionIndex].Contains(x.Index)).ToList();
            for (int i = 0; i < result.Count; i++)
            {
                if (result[i].Type == RegionType.Stable)
                {
                    //Определяем через какую точку мы войдем в регион
                    NavMeshHit hit;
                    if (NavMesh.SamplePosition(inregionPosition, out hit, 200, result[i].Index))
                    {
                        result[i].PathPoints = new List<Vector3> { hit.position };
                    }
                }
                else if (result[i].Type == RegionType.Moving)
                {
                    result[i].PathPoints = new List<Vector3> { ((MovingRegion)result[i]).EntryPoint.transform.position };
                }
                result[i].Price = GetMovementPrice(result[i], regionIndex);
            }
            return result;
        }

        private float GetMovementPrice(Region region, int triggerRegionIndex)
        {
            //Если тип региона обычный, то просто берем минимальное время прохождения региона (минимальная ширина на скорость, полагаю)
            //Если тип региона - движущийся, то мы берем текущее положение точки входа, 
            //и смотрим время, через которое она совершит оборот сначала до первого положения входа в зону движения,
            //потом полуоборот до второго положения входа
            return 0;
        }
    }
}
