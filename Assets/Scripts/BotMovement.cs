using Assets.Scripts.AI.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BotMovement : MonoBehaviour
{
    public GameObject terrain;

    public GameObject DEBUGSHIT;

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
        //  Если есть текущая целевая точка
        if(currentTarget != null)
        {
            float distanceToTarget = currentTarget.Distance(transform.position);
            //  Если до текущей целевой точки ещё далеко, то выходим
            if (distanceToTarget >= movementProperties.epsilon)//|| currentTarget.TimeMoment - Time.fixedTime > movementProperties.epsilon) 
                return true;
            //  Иначе удаляем её из маршрута и берём следующую
            Debug.Log("Point reached : " + Time.fixedTime.ToString());
            currentPath.RemoveAt(0);
            if (currentPath.Count > 0) 
            {
                //  Берём очередную точку и на выход
                currentTarget = currentPath[0];
                return true;
            }
            else
            {
                currentTarget = null;
                currentPath = null;
                //  А вот тут надо будет проверять, есть ли уже построенный маршрут
            }
        }
        else 
            if (targetUpdated && globalTarget != null)
        {
            targetUpdated = false;
            currentPath = LocalPlanner.GetLocalRoute(globalTarget, new BaseAI.PathNode(transform.position), movementProperties);
            for (int i = 0; i < currentPath.Count; i++)
            {
                Instantiate(DEBUGSHIT, currentPath[i].Position, Quaternion.identity);
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
            if(plannedPath != null)
            {
                currentPath = plannedPath;
                plannedPath = null;
                if (currentPath.Count > 0)
                    currentTarget = currentPath[0];
            }
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
