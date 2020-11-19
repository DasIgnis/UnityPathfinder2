using BaseAI;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine.AI;

namespace Assets.Scripts.AI.Pathfinding
{
    public static class GlobalPlanner
    {
        public static PathNode GetGlobalRoute(PathNode target, PathNode position)
        {
            NavMeshHit currentArea;
            if (!NavMesh.SamplePosition(position.Position, out currentArea, 0.1f, NavMesh.AllAreas))
                return null;
            NavMeshHit targetArea;
            if (!NavMesh.SamplePosition(target.Position, out targetArea, 0.1f, NavMesh.AllAreas))
                return null;
            if (currentArea.mask == targetArea.mask)
                return target;
            return null;
        }
    }
}
