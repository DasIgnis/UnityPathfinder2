using BaseAI;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.Scripts.AI.Extensions
{
    public static class Extensions
    {
        public static bool EqualsSigma(this PathNode point, PathNode other, float sigma)
        {
            return Math.Abs(point.Position.x - other.Position.x) < sigma
                && Math.Abs(point.Position.z - other.Position.z) < sigma;
        }
    }
}
