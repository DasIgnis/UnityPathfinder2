using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Класс, созданный с одной целью - генерация отладочного списка точек для движения бота, и не более
/// </summary>
public class DebugHelper : MonoBehaviour
{
    /// <summary>
    /// Ссылка на объект, отвечающий за движение бота
    /// </summary>
    [SerializeField] private BotMovement movementSript;
    
    /// <summary>
    /// Список объектов, через которые бот будет двигаться - выставляются в Unity для отладки
    /// </summary>
    [SerializeField] private List<GameObject> wayPoints;

    // На старте однократно определить список точек маршрута
    void Start()
    {
        //  Блокируем объект для исключительного доступа (вообще-то в этом нет необходимости, но для порядка)
        //lock (movementSript) {
        //    //  Для каждой точки списка формируем объект класса PathNode
        //    movementSript.plannedPath = new List<BaseAI.PathNode>();
        //    //foreach (var point in wayPoints)
        //      //  movementSript.plannedPath.Add(new BaseAI.PathNode(point.transform.position));
        //    for (int i = 0; i < wayPoints.Count; ++i) 
        //    {
        //        movementSript.plannedPath.Add(new BaseAI.PathNode(wayPoints[i].transform.position));
        //        movementSript.plannedPath[movementSript.plannedPath.Count - 1].TimeMoment = 5.3f * i;
        //    }
        //}

    }
}
