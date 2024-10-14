#ifndef StateMachine_HPP
#define StateMachine_HPP

/**
 * @class StateMachine
 * @brief 状态机基类
 */
 
#ifdef __cplusplus

class StateMachine
{
public:
    /**
     * @brief 初始化状态机
     */
    virtual void init() {};
    /**
     * @brief 进入状态
     */
    virtual void enter() {};
    /**
     * @brief 执行状态
     */
    virtual void execute() {};
    /**
     * @brief 退出状态
     */
    virtual void exit() {};
    /**
     * @brief 运行状态机
     *
     * 该函数会在每个循环中被调用，用于执行状态机的逻辑。上面的三个函数enter, execute, exit会循环调用。
     */
    virtual void run() {};

    StateMachine() {};
    ~StateMachine() {};
};




#endif
#endif // StateMachine_HPP
