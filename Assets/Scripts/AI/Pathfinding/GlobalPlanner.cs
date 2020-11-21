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
        public static PathNode GetGlobalRoute(PathNode target, PathNode position)
        {
            NavMeshHit targetArea;
            if (!NavMesh.SamplePosition(target.Position, out targetArea, 1f, NavMesh.AllAreas))
                return null;

            NavMeshHit currentArea;
            Vector3 adjustedPosition = new Vector3(position.Position.x, targetArea.position.y, position.Position.z);
            if (!NavMesh.SamplePosition(adjustedPosition, out currentArea, 1f, NavMesh.AllAreas))
                return null;

            if (currentArea.mask == targetArea.mask)
                return target;

            //Если мы в точке выхода текущей зоны, то вернуть точку входа следующей зоны
            
            //TODO:
            //Здесь берем список всех зон, для каждой вычисляем цену прохождения и строим оптимальный маршрут
            //Ценой пусть будет время, которое мы затратим для прохождения. Соответственно для простых зон что-то вроде 
            //расстояние от входа до выхода / скорость, 
            //для зоны с платформой время, за которое она повернется от её позиции на момент достижения предыдущей границы
            //до границы следующей за ней зоны

            //Соответственно, для каждой зоны мы знаем точку входа и точку выхода, их на карте можно просто поставить в виде кружочков и прописать в явном виде
            //Для платформы нужна динамическая точка входа, либо точка входа не динамическая, но с таймаутом - когда мы сможем прыгнуть
            //После того, как мы получили маршрут по зонам, возвращаем ближайшую точку входа
            //Возможно имеет смысл мемоизировать построенный маршрут

            

            return null;
        }
    }
}
