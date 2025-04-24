#pragma once

class Parameters
{
public:
    enum ParamType
    {
        PARAM_FLOAT,
        PARAM_INT
    };

    struct Parameter
    {
        ParamType type;
        const char *name;

        union
        {
            float f;
            int i;
        } value;
    };

    const static Parameter *getParamByName(const char *name);
    static bool getFloat(const char *name, float &out);
    static bool getInt(const char *name, int &out);

private:
    const static Parameter params[];
};
