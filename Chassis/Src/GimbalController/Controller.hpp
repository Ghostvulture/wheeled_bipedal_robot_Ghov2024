#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

/**
 * @class Controller
 * @brief 控制器基类，提供控制器的基本接口。
 */
#ifdef __cplusplus

class Controller
{
public:
    /**
     * @brief 运行控制器
     */
    virtual void run() = 0;

    Controller() {};
    ~Controller() {};
};

#endif

#endif // CONTROLLER_HPP
