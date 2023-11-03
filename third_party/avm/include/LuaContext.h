/************************************************************************
 * \class LuaContext
 *
 * \ingroup GroupName
 *
 * \brief 
 *
 * Lua 
 *
 * \note 
 *
 *
 * \version 1.0
 *
 *
 *
************************************************************************/
#ifndef _LUACONTEXT_H_
#define _LUACONTEXT_H_
#include <string.h>
#include <stdarg.h>

struct lua_State;
typedef int(*lua_CFunction) (lua_State *L);

#define MAX_NAME_LENGTH 2048

#ifdef LUA_EXPORTS
#define API_SPEC __declspec(dllexport)
#else
#ifdef LUA_IMPORTS
#define API_SPEC __declspec(dllimport)
#else
#define API_SPEC
#endif
#endif

class API_SPEC LuaContext
{
public:
	enum LUA_VALUE_TYPE
	{
		LVT_NIL,
		LVT_BOOLEAN,
		LVT_POINTER,
		LVT_NUMBER,
		LVT_STRING,
		LVT_TABLE,
		LVT_FUNCTION,
		LVT_USER_POINTER,
		LVT_INT = 0x10,
	};

public:
	LuaContext();
	LuaContext(lua_State* pState);
	LuaContext(const LuaContext &other);
	virtual ~LuaContext();

	LuaContext* CreateNewThread();
	//file operation
	bool DoFile(const char* fileName);
	bool DoString(const char* scriptString);

	//table operation
	bool BeginTable(const char *key, int idx, bool createWhenNil = false);
	void EndTable();
	void EndAllTable();

	bool SetTableMeta(const char *metatableName, int idx);
	int GetTableArraySize();
	void GetTableKeys(const char** keys, int* indexes, int &keyCount, int& indexCount);

	const char* GetCurrentFullKey() const;

	int GetType(const char *key, int idx);
	bool IsInt(const char *key, int idx);

	bool GetBoolean(const char *key, int idx, bool defval = true);
	long long GetInt(const char *key, int idx, long long defval = 0);
	double GetNumber(const char *key, int idx, double defval = 0.0);
	const char* GetString(const char *key, int idx, const char* defval = NULL);
	void* GetUserData(const char *key, int idx);

	void SetBoolean(const char *key, int idx, bool value);
	void SetInt(const char *key, int idx, long long value);
	void SetNumber(const char *key, int idx, double value);
	void SetString(const char *key, int idx, const char* value);
	void SetFunction(const char *key, int idx, const char* funcName);
	void SetFunction(const char *key, int idx, lua_CFunction pFunction);
	void SetUserData(const char *key, int idx, void* pUserData);
	void SetNil(const char *key, int idx);
	// Create a userdata in current table and return it
	void* NewUserData(const char *key, int idx, unsigned int size);

	//stack operation
	void DumpStack();
	double GetStackNumber(int stackIndex, double defVal = 0.0);
	long long GetStackInt(int stackIndex, long long defVal = 0);
	bool GetStackBoolean(int stackIndex, bool defVal = false);
	const char* GetStackString(int stackIndex, const char* defVal = NULL);
	void* GetStackUserData(int stackIndex, void* defVal = NULL);
	int GetStackType(int stackIndex);
	void* AddStackUserData(int size);
	void AddStackPointer(void* pData);
	void AddStackTable();
	void AddStackInt(long long value);
	void AddStackBoolean(bool value);
	void AddStackString(const char* value);
	void AddStackNumber(double value);
	void AddStackNil();
	void PopStack(int n);

	//registry operation
	void SetRegistyUserData(const char* key, void* userData);
	void* GetRegistryUserData(const char* key);

	//function operation
	void RegisterFunction(const char* functionName, lua_CFunction pFunction);
	bool CallFunction(const char* functionCall, ...);
	bool ExecFunction(const char* functionCall, va_list args);
	void GarbageCollect();
	static LuaContext* GetContext(lua_State* pLuaContext);

private:
	int GetTableType(const char *key, int idx);
	bool IsTableInt(const char *key, int idx);

	bool GetTableBoolean(const char *key, int idx, bool defval = true);
	long long GetTableInt(const char *key, int idx, long long defval = 0);
	double GetTableNumber(const char *key, int idx, double defval = 0.0);
	const char* GetTableString(const char *key, int idx, const char* defval = NULL);
	void* GetTableUserData(const char *key, int idx);

	void SetTableBoolean(const char *key, int idx, bool value);
	void SetTableInt(const char *key, int idx, long long value);
	void SetTableNumber(const char *key, int idx, double value);
	void SetTableString(const char *key, int idx, const char* value);
	void SetTableFunction(const char *key, int idx, const char* funcName);
	void SetTableFunction(const char *key, int idx, lua_CFunction pFunction);
	void SetTableUserData(const char *key, int idx, void* pUserData);
	void SetTableNil(const char *key, int idx);
	// Create a userdata in current table and return it
	void* NewTableUserData(const char *key, int idx, unsigned int size);

	//global operation
	void SetGlobalInt(const char *key, long long val);
	void SetGlobalNumber(const char *key, double val);
	void SetGlobalBoolean(const char *key, bool val);
	void SetGlobalString(const char *key, const char* val);
	void SetGlobalFunction(const char *key, const char* funcName);
	void SetGlobalFunction(const char *key, lua_CFunction pFunction);
	void SetGlobalUserData(const char *key, void* pUserData);
	void SetGlobalNil(const char *key);
	void* NewGlobalUserData(const char *key, unsigned int size);

	bool IsGlobalInt(const char *key);
	int GetGlobalType(const char *key);
	bool GetGlobalBoolean(const char *key, bool defval);
	long long GetGlobalInt(const char *key, long long defval);
	double GetGlobalNumber(const char *key, double defval);
	const char* GetGlobalString(const char *key, const char* defval);
	void* GetGlobalUserData(const char *key);

protected:
	void InitState();
	void ReleaseState();
	bool BeginGlobalTable(const char *key, int idx, bool createWhenNil = false);
	bool BeginSubTable(const char *key, int idx, bool createWhenNil = false);

	char m_currentTableSpace[MAX_NAME_LENGTH];
	int m_tableLevel;
	lua_State* m_state;
	bool m_majorState;

private:
	const LuaContext &operator=(const LuaContext &other);
};

#endif
