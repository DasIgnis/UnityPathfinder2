using Assets.Scripts.AI.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class BotMovement : MonoBehaviour
{
    public GameObject terrain;

    public GameObject DEBUG;

    private GlobalPlanner globalPlanner = new GlobalPlanner();

    /// <summary>
    /// Запланированный путь как список точек маршрута
    /// </summary>
    [SerializeField] public List<BaseAI.PathNode> plannedPath;
    
    /// <summary>
    /// Текущий путь как список точек маршрута
    /// </summary>
    [SerializeField] List<BaseAI.PathNode> currentPath = null;

    /// <summary>
    /// Текущая целевая точка, к которой идёт бот
    /// </summary>
    private BaseAI.PathNode currentTarget = null;

    private BaseAI.PathNode globalTarget = null;
    private bool targetUpdated = false;
    
    /// <summary>
    /// Параметры движения бота
    /// </summary>
    [SerializeField] private BaseAI.MovementProperties movementProperties;

    public float obstacleRange = 5.0f;
    public int steps = 0;
    private float leftLegAngle = 3f;
    [SerializeField] private bool walking = false;


    //  Сила, тянущая "вверх" упавшего бота и заставляющая его вставать
    [SerializeField] float force = 5.0f;
    //  Угол отклонения, при котором начинает действовать "поднимающая" сила
    [SerializeField] float max_angle = 20.0f;

    [SerializeField] private GameObject leftLeg = null;
    [SerializeField] private GameObject rightLeg = null;
    [SerializeField] private GameObject leftLegJoint = null;
    [SerializeField] private GameObject rightLegJoint = null;

    // Start is called before the first frame update
    void Start()
    {
        
    }
    /// <summary>
    /// Движение ног - болтаем туда-сюда
    /// </summary>
    void MoveLegs()
    {
        //  Движение ножек сделать
        if (steps >= 20)
        {
            leftLegAngle = -leftLegAngle;
            steps = -20;
        }
        steps++;

        leftLeg.transform.RotateAround(leftLegJoint.transform.position, transform.right, leftLegAngle);
        rightLeg.transform.RotateAround(rightLegJoint.transform.position, transform.right, -leftLegAngle);
    }
    /// <summary>
    /// Обновление текущей целевой точки - куда вообще двигаться
    /// </summary>
    private bool UpdateCurrentTargetPoint()
    {
        if(currentTarget != null)
        {
            Vector3 dummyPosition = new Vector3(transform.position.x, currentTarget.Position.y, transform.position.z);
            float distanceToTarget = currentTarget.Distance(dummyPosition);
            if (distanceToTarget >= movementProperties.epsilon)//|| currentTarget.TimeMoment - Time.fixedTime > movementProperties.epsilon) 
                return true;
            //Debug.Log("Point reached : " + Time.fixedTime.ToString());
            if (currentPath != null)
            {
                if (currentPath.Count > 0) currentPath.RemoveAt(0);
                if (currentPath.Count > 0)
                {
                    currentTarget = currentPath[0];
                    return true;
                }
            }
            else
            {
                currentTarget = null;
                // TODO:
                // Если мы сюда пришли, то мы находимся либо в цели, либо на границе регионов, куда нас направил глобальный планировщик
                // Снова дёргаем планировщика, если вернул не null, то 
                // 1) смотрим, в той же что и мы зоне вернувшаяся точка
                // 2) если в той же, то скармливает её локалпланнеру и возвращаем этот маршрут
                // 3) если нет, то мы на границе зон. Сразу присваиваем точку из глобального планировщика currentTarget
                // Не исключено, что из этой ф-ции имеет смысл возвращать не бульку, а enum со статусом работы
                var currentPathNode = new BaseAI.PathNode(transform.position);
                var milestone = globalPlanner.GetGlobalRoute(globalTarget, currentPathNode, movementProperties);
                if (milestone != null)
                {
                    NavMeshHit currentArea;
                    Vector3 adjustedPosition = new Vector3(transform.position.x, milestone.Position.y, transform.position.z);
                    if (NavMesh.SamplePosition(adjustedPosition, out currentArea, 1f, NavMesh.AllAreas))
                    {
                        if (currentArea.mask == milestone.RegionId)
                        {
                            currentPath = LocalPlanner.GetLocalRoute(milestone, currentPathNode, movementProperties);
                            //Для дебага и прочих шалостей
                            for (int i = 0; i < currentPath.Count; i++)
                            {
                                Instantiate(DEBUG, currentPath[i].Position, Quaternion.identity);
                            }
                        }
                        else
                        {
                            currentTarget = milestone;
                            return true;
                        }
                    }
                }
            }
        }
        else 
            if (targetUpdated && globalTarget != null)
        {
            targetUpdated = false;
            var currentPathNode = new BaseAI.PathNode(transform.position);
            //Пример получения майлстоуна от планировщика
            var milestone = globalPlanner.GetGlobalRoute(globalTarget, currentPathNode, movementProperties);
            if (milestone != null)
            {
                currentPath = LocalPlanner.GetLocalRoute(milestone, currentPathNode, movementProperties);
                //Для дебага и прочих шалостей
                for (int i = 0; i < currentPath.Count; i++)
                {
                    Instantiate(DEBUG, currentPath[i].Position, Quaternion.identity);
                }
            }
        }

        if(currentPath != null)
        {
            if(currentPath.Count > 0 )
            {
                currentTarget = currentPath[0];
                return true;
            }
            else
            {
                currentPath = null;
            }
        }

        //  Здесь мы только в том случае, если целевой нет, и текущего пути нет - и то, и другое null
        //  Обращение к plannedPath желательно сделать через блокировку - именно этот список задаётся извне планировщиком
        //  Непонятно, насколько lock затратен, можно ещё булевский флажок добавить, его сначала проверять
        ////lock(plannedPath)
        //{
            //if(plannedPath != null)
            //{
            //    currentPath = plannedPath;
            //    plannedPath = null;
            //    if (currentPath.Count > 0)
            //        currentTarget = currentPath[0];
            //}
        //}
        return currentTarget != null;
    }

    /// <summary>
    /// Очередной шаг бота - движение
    /// </summary>
    /// <returns></returns>
    bool MoveBot()
    {
        //  Выполняем обновление текущей целевой точки
        if (!UpdateCurrentTargetPoint())
        {
            //  Это ситуация когда идти некуда - цели нет
            return false;
        }

        //  Ну у нас тут точно есть целевая точка, вот в неё и пойдём
        //  Определяем угол поворота, и расстояние до целевой
        Vector3 directionToTarget = currentTarget.Position - transform.position;
        float angle = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);
        //  Теперь угол надо привести к допустимому диапазону
        angle = Mathf.Clamp(angle, -movementProperties.rotationAngle, movementProperties.rotationAngle);

        //  Зная угол, нужно получить направление движения (мы можем сразу не повернуть в сторону цели)
        //  Выполняем вращение вокруг оси Oy

        //  Угол определили, теперь, собственно, определяем расстояние для шага
        float stepLength = directionToTarget.magnitude;
        float actualStep = Mathf.Clamp(stepLength, 0.0f, movementProperties.maxSpeed * Time.deltaTime);
        //  Поворот может быть проблемой, если слишком близко подошли к целевой точке
        //  Надо как-то следить за скоростью, она не может превышать расстояние до целевой точки???
        transform.Rotate(Vector3.up, angle);

        //  Время прибытия - оставшееся время
        //var remainedTime = currentTarget.TimeMoment - Time.fixedTime;
        //if (remainedTime < movementProperties.epsilon)
        //{
        transform.position = transform.position + actualStep * transform.forward;
        //}
        //else
        //{
        //    //  Дедлайн ещё не скоро!!! Стоим спим
        //    if (currentTarget.Distance(transform.position) < movementProperties.epsilon) return true;

        //    transform.position = transform.position + actualStep * transform.forward / remainedTime;
        //}
        return true;
    }


    /// <summary>
    /// Вызывается каждый кадр
    /// </summary>
    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            RaycastHit hit;
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (terrain.GetComponent<Collider>().Raycast(ray, out hit, Mathf.Infinity))
            {
                globalTarget = new BaseAI.PathNode(hit.point);
                targetUpdated = true;
                Instantiate(DEBUG, globalTarget.Position, Quaternion.identity);
            }
        }

        //  Фрагмент кода, отвечающий за вставание
        var vertical_angle = Vector3.Angle(Vector3.up, transform.up);
        if (vertical_angle > max_angle)
        {
            GetComponent<Rigidbody>().AddForceAtPosition(5 * force * Vector3.up, transform.position + 3.0f * transform.up, ForceMode.Force);
        };

        if (!walking) return;

        //  Собственно движение
        if (MoveBot())
            MoveLegs();
    }
}
