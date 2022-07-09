from typing import List

from dataclasses import dataclass, field

from cxxheaderparser.parser import CxxParser

# 'Fixing' complains about typedefs
CxxParser._fundamentals.discard("wchar_t")

from cxxheaderparser.types import (
    EnumDecl,
    Field,
    ForwardDecl,
    FriendDecl,
    Function,
    Method,
    Typedef,
    UsingAlias,
    UsingDecl,
    Variable,
    Pointer,
    Type,
    PQName,
    NameSpecifier,
    FundamentalSpecifier,
    Parameter,
    Array,
    Value,
    Token,
    FunctionType,
)

from cxxheaderparser.parserstate import (
    State,
    EmptyBlockState,
    ClassBlockState,
    ExternBlockState,
    NamespaceBlockState,
)


@dataclass(frozen=True)
class ApiEntryFunction:
    name: str
    ret_type: str
    args: str


@dataclass(frozen=True)
class ApiEntryVariable:
    name: str
    obj_type: str


@dataclass
class ApiEntries:
    # These are sets, to avoid creating duplicates when we have multiple
    # declarations with same signature
    functions: List[ApiEntryFunction] = field(default_factory=set)
    variables: List[ApiEntryVariable] = field(default_factory=set)


class ApiManager:
    def __init__(self):
        self.api = ApiEntries()
        # self.root_headers = []
        self.name_hashes = set()

    # Calculate hash of name and raise exception if it already is in the set
    def _name_check(self, name: str):
        name_hash = gnu_sym_hash(name)
        if name_hash in self.name_hashes:
            raise Exception(f"Hash collision on {name}")
        self.name_hashes.add(name_hash)

    def add_function(self, function_def: ApiEntryFunction):
        if function_def in self.api.functions:
            return
        self._name_check(function_def.name)
        self.api.functions.add(function_def)

    def add_variable(self, variable_def: ApiEntryVariable):
        if variable_def in self.api.variables:
            return
        self._name_check(variable_def.name)
        self.api.variables.add(variable_def)


def gnu_sym_hash(name: str):
    h = 0x1505
    for c in name:
        h = (h << 5) + h + ord(c)
    return str(hex(h))[-8:]


class Sdk:
    def __init__(self):
        self.api_manager = ApiManager()

    def process_source_file_for_sdk(self, file_path: str):
        visitor = SdkCxxVisitor(self.api_manager)
        with open(file_path, "rt") as f:
            content = f.read()
        parser = CxxParser(file_path, content, visitor, None)
        parser.parse()

    def get_functions(self):
        return sorted(self.api_manager.api.functions, key=lambda o: o.name)

    def get_variables(self):
        return sorted(self.api_manager.api.variables, key=lambda o: o.name)


def stringify_array_dimension(size_descr):
    if not size_descr:
        return ""
    return stringify_descr(size_descr)


def stringify_array_descr(type_descr):
    assert isinstance(type_descr, Array)
    return (
        stringify_descr(type_descr.array_of),
        stringify_array_dimension(type_descr.size),
    )


def stringify_descr(type_descr):
    if isinstance(type_descr, (NameSpecifier, FundamentalSpecifier)):
        return type_descr.name
    elif isinstance(type_descr, PQName):
        return "::".join(map(stringify_descr, type_descr.segments))
    elif isinstance(type_descr, Pointer):
        # Hack
        if isinstance(type_descr.ptr_to, FunctionType):
            return stringify_descr(type_descr.ptr_to)
        return f"{stringify_descr(type_descr.ptr_to)}*"
    elif isinstance(type_descr, Type):
        return (
            f"{'const ' if type_descr.const else ''}"
            f"{'volatile ' if type_descr.volatile else ''}"
            f"{stringify_descr(type_descr.typename)}"
        )
    elif isinstance(type_descr, Parameter):
        return stringify_descr(type_descr.type)
    elif isinstance(type_descr, Array):
        # Hack for 2d arrays
        if isinstance(type_descr.array_of, Array):
            argtype, dimension = stringify_array_descr(type_descr.array_of)
            return (
                f"{argtype}[{stringify_array_dimension(type_descr.size)}][{dimension}]"
            )
        return f"{stringify_descr(type_descr.array_of)}[{stringify_array_dimension(type_descr.size)}]"
    elif isinstance(type_descr, Value):
        return " ".join(map(stringify_descr, type_descr.tokens))
    elif isinstance(type_descr, FunctionType):
        return f"{stringify_descr(type_descr.return_type)} (*)({', '.join(map(stringify_descr, type_descr.parameters))})"
    elif isinstance(type_descr, Token):
        return type_descr.value
    elif type_descr is None:
        return ""
    else:
        raise Exception("unsupported type_descr: %s" % type_descr)


class SdkCxxVisitor:
    def __init__(self, api_manager: ApiManager):
        self.api = api_manager

    def on_variable(self, state: State, v: Variable) -> None:
        # print("on_variable", v)
        if not v.extern:
            # print("SKIPPING VAR", v)
            return
        self.api.add_variable(
            ApiEntryVariable(
                stringify_descr(v.name),
                stringify_descr(v.type),
            )
        )

    def on_function(self, state: State, fn: Function) -> None:
        # print("on_function", fn)
        if fn.inline or fn.has_body:
            # print("SKIPPING FN", fn.name)
            return
        self.api.add_function(
            ApiEntryFunction(
                stringify_descr(fn.name),
                stringify_descr(fn.return_type),
                ", ".join(map(stringify_descr, fn.parameters))
                + (", ..." if fn.vararg else ""),
            )
        )

    def on_define(self, state: State, content: str) -> None:
        pass

    def on_pragma(self, state: State, content: str) -> None:
        pass

    def on_include(self, state: State, filename: str) -> None:
        pass

    def on_empty_block_start(self, state: EmptyBlockState) -> None:
        pass

    def on_empty_block_end(self, state: EmptyBlockState) -> None:
        pass

    def on_extern_block_start(self, state: ExternBlockState) -> None:
        pass

    def on_extern_block_end(self, state: ExternBlockState) -> None:
        pass

    def on_namespace_start(self, state: NamespaceBlockState) -> None:
        pass

    def on_namespace_end(self, state: NamespaceBlockState) -> None:
        pass

    def on_forward_decl(self, state: State, fdecl: ForwardDecl) -> None:
        pass

    def on_typedef(self, state: State, typedef: Typedef) -> None:
        pass

    def on_using_namespace(self, state: State, namespace: List[str]) -> None:
        pass

    def on_using_alias(self, state: State, using: UsingAlias) -> None:
        pass

    def on_using_declaration(self, state: State, using: UsingDecl) -> None:
        pass

    def on_enum(self, state: State, enum: EnumDecl) -> None:
        pass

    def on_class_start(self, state: ClassBlockState) -> None:
        pass

    def on_class_field(self, state: State, f: Field) -> None:
        pass

    def on_class_method(self, state: ClassBlockState, method: Method) -> None:
        pass

    def on_class_friend(self, state: ClassBlockState, friend: FriendDecl) -> None:
        pass

    def on_class_end(self, state: ClassBlockState) -> None:
        pass