# Copyright 2022 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Represents the types of TF functions."""

import collections
import inspect
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple

from tensorflow.core.function import trace_type
from tensorflow.python.types import trace

# Represents a defined parameter default value that is saved alongside the
# function's captures.
CAPTURED_DEFAULT_VALUE = object()


class Parameter(inspect.Parameter):
  """Represents a parameter to a function."""

  def __init__(self, name: str, kind: Any, optional: bool,
               type_constraint: Optional[trace.TraceType]):
    if optional and kind not in [
        self.POSITIONAL_ONLY, self.KEYWORD_ONLY, self.POSITIONAL_OR_KEYWORD
    ]:
      raise ValueError(
          "Parameter " + name +
          " is optional and its kind must be one of {POSITIONAL_ONLY, " +
          "KEYWORD_ONLY, POSITIONAL_OR_KEYWORD}. Got: " + str(kind))

    if type_constraint and kind in [self.VAR_POSITIONAL, self.VAR_KEYWORD]:
      raise TypeError("Variable args/kwargs can not have type constraints.")

    if not isinstance(type_constraint, (trace.TraceType, type(None))):
      raise TypeError(
          "Type constraints can only be an instance of a TraceType but got " +
          "type_constraint=" + str(type_constraint) + " for Parameter " + name)

    super().__init__(
        name,
        kind,
        default=CAPTURED_DEFAULT_VALUE if optional else self.empty,
        annotation=type_constraint
        if type_constraint is not None else self.empty)

  @property
  def optional(self) -> bool:
    """If this parameter might not be supplied for a call."""
    return self.default is not self.empty

  @property
  def type_constraint(self) -> Optional[trace.TraceType]:
    """A supertype that the parameter's type must subtype for validity."""
    return self.annotation if self.annotation is not self.empty else None

  def is_subtype_of(self, other: "Parameter") -> bool:
    """Returns True if self is a supertype of other Parameter."""
    if not self.type_constraint or not other.type_constraint:
      raise TypeError(
          "Can not determine relationship between partially specified types.")

    if ((self.name, self.kind, self.optional) !=
        (other.name, other.kind, other.optional)):
      return False

    return self.type_constraint.is_subtype_of(other.type_constraint)

  def most_specific_common_supertype(
      self, others: Sequence["Parameter"]) -> Optional["Parameter"]:
    """Returns a common supertype (if exists)."""
    if not self.type_constraint or any(
        not other.type_constraint for other in others):
      raise TypeError(
          "Can not determine relationship between partially specified types.")

    for other in others:
      if ((self.name, self.kind, self.optional) !=
          (other.name, other.kind, other.optional)):
        return None

    supertyped_constraint = self.type_constraint.most_specific_common_supertype(
        [other.type_constraint for other in others])
    if supertyped_constraint:
      return Parameter(self.name, self.kind, self.optional,
                       supertyped_constraint)
    else:
      return None

  def __eq__(self, other: Any) -> bool:
    if not isinstance(other, Parameter):
      return NotImplemented

    return ((self.name, self.kind, self.optional,
             self.type_constraint) == (other.name, other.kind, other.optional,
                                       other.type_constraint))

  def __hash__(self):
    return hash((self.name, self.kind, self.optional, self.type_constraint))

  def __repr__(self):
    return ("Parameter(name=" + self.name + ", kind=" + str(self.kind) +
            ", optional=" + repr(self.optional) + ", type_constraint=" +
            repr(self.type_constraint) + ")")

  def __reduce__(self):
    return (self.__class__, (self.name, self.kind, self.optional,
                             self.type_constraint))


class FunctionType(inspect.Signature):
  """Represents the signature of a polymorphic/monomorphic function."""

  def __init__(self,
               parameters: Sequence[inspect.Parameter],
               captures: Optional[collections.OrderedDict] = None,
               **kwargs):
    super().__init__(parameters, **kwargs)
    self._captures = captures if captures else collections.OrderedDict()

  @property
  def parameters(self) -> Mapping[str, Any]:
    return super().parameters

  @property
  def captures(self) -> collections.OrderedDict:
    return self._captures

  # TODO(fmuham): Use this method instead of fullargspec and tf_inspect.
  @classmethod
  def from_callable(cls,
                    obj: Callable[..., Any],
                    *,
                    follow_wrapped: bool = True) -> "FunctionType":
    """Generate FunctionType from a python Callable."""
    signature = super().from_callable(obj, follow_wrapped=follow_wrapped)
    # TODO(fmuham): Support TraceType-based annotations.
    parameters = [
        Parameter(p.name, p.kind, p.default is not p.empty, None)
        for p in signature.parameters.values()
    ]

    return FunctionType(parameters)

  @classmethod
  def get_default_values(cls,
                         obj: Callable[..., Any],
                         *,
                         follow_wrapped: bool = True) -> Dict[str, Any]:
    """Inspects and returns a dictionary of default values."""
    signature = super().from_callable(obj, follow_wrapped=follow_wrapped)
    default_values = {}
    for p in signature.parameters.values():
      if p.default is not p.empty:
        default_values[p.name] = p.default
    return default_values

  def is_supertype_of(self, other: "FunctionType") -> bool:
    """Returns True if self is a supertype of other FunctionType."""
    if len(self.parameters) != len(other.parameters):
      return False

    for self_param, other_param in zip(self.parameters.values(),
                                       other.parameters.values()):
      # Functions are contravariant on their parameter types.
      if not self_param.is_subtype_of(other_param):
        return False

    # Self must have all capture names of other.
    if not all(name in self.captures for name in other.captures):
      return False

    # Functions are contravariant upon the capture types.
    return all(self.captures[name].is_subtype_of(capture_type)
               for name, capture_type in other.captures.items())

  def most_specific_common_subtype(
      self, others: Sequence["FunctionType"]) -> Optional["FunctionType"]:
    """Returns a common subtype (if exists)."""
    subtyped_parameters = []

    for i, parameter in enumerate(self.parameters.values()):
      # Functions are contravariant on their parameter types.
      subtyped_parameter = parameter.most_specific_common_supertype(
          [list(other.parameters.values())[i] for other in others])
      if subtyped_parameter is None:
        return None
      subtyped_parameters.append(subtyped_parameter)

    if not all(subtyped_parameters):
      return None

    # Common subtype must use captures common to all.
    capture_names = set(self.captures.keys())
    for other in others:
      capture_names = capture_names.intersection(other.captures.keys())

    subtyped_captures = collections.OrderedDict()
    for name in capture_names:
      # Functions are contravariant upon the capture types.
      common_type = self.captures[name].most_specific_common_supertype(
          [other.captures[name] for other in others])
      if common_type is None:
        return None
      else:
        subtyped_captures[name] = common_type

    return FunctionType(subtyped_parameters, subtyped_captures)

  def placeholder_arguments(self) -> inspect.BoundArguments:
    """Returns BoundArguments of values that can be used for tracing."""
    placeholder_context = trace_type.InternalPlaceholderContext()
    arguments = collections.OrderedDict()
    for parameter in self.parameters.values():
      if parameter.kind in {Parameter.VAR_POSITIONAL, Parameter.VAR_KEYWORD}:
        raise ValueError("Can not generate placeholder values for "
                         "variable length function type.")

      if not parameter.type_constraint:
        raise ValueError("Can not generate placeholder value for "
                         "partially defined function type.")

      arguments[parameter.name] = parameter.type_constraint._placeholder_value(  # pylint: disable=protected-access
          placeholder_context)

    return inspect.BoundArguments(self, arguments)

  def __eq__(self, other: Any) -> bool:
    if not isinstance(other, FunctionType):
      return NotImplemented

    return (self.parameters, self.captures) == (other.parameters,
                                                other.captures)

  def __hash__(self) -> int:
    return hash((tuple(self.parameters.items()), tuple(self.captures.items())))

  def __repr__(self):
    return (f"FunctionType(parameters={list(self.parameters.values())!r}, "
            f"captures={self.captures})")


# TODO(fmuham): Raise warning here when load-bearing.
# In future, replace warning with exception.
def sanitize_arg_name(name: str) -> str:
  """Sanitizes Spec names.

  Matches Graph Node and Python naming conventions.

  Without sanitization, names that are not legal Python parameter names can be
  set which makes it challenging to represent callables supporting the named
  calling capability.

  Args:
    name: The name to sanitize.

  Returns:
    A string that meets Python parameter conventions.
  """
  # Lower case and replace non-alphanumeric chars with '_'
  swapped = "".join([c if c.isalnum() else "_" for c in name.lower()])

  if swapped[0].isalpha():
    return swapped
  else:
    return "arg_" + swapped


# TODO(fmuham): Consider forcing kind to be always POSITIONAL_OR_KEYWORD.
def _make_validated_mono_param(name, value, kind, type_context, poly_type):
  """Generates and validates a parameter for Monomorphic FunctionType."""
  mono_type = trace_type.from_value(value, type_context)

  if poly_type and not mono_type.is_subtype_of(poly_type):
    raise TypeError(f"Parameter {name} was expected to be of type "
                    f"{poly_type} but is {mono_type}")

  return Parameter(name, kind, False, mono_type)


def canonicalize_to_monomorphic(
    args: Tuple[Any, ...], kwargs: Dict[Any, Any], default_values: Dict[Any,
                                                                        Any],
    captures: Dict[Any, Any], polymorphic_type: FunctionType
) -> Tuple[inspect.BoundArguments, FunctionType,
           trace_type.InternalTracingContext]:
  """Converts polymorphic parameters to monomorphic and associated type."""
  poly_bound_arguments = polymorphic_type.bind(*args, **kwargs)
  poly_bound_arguments.apply_defaults()

  # Inject Default Values.
  default_values_injected = poly_bound_arguments.arguments
  for name, value in default_values_injected.items():
    if value is CAPTURED_DEFAULT_VALUE:
      default_values_injected[name] = default_values[name]
  poly_bound_arguments = inspect.BoundArguments(poly_bound_arguments.signature,
                                                default_values_injected)

  parameters = []
  type_context = trace_type.InternalTracingContext()
  has_var_positional = any(p.kind is Parameter.VAR_POSITIONAL
                           for p in polymorphic_type.parameters.values())

  for name, arg in poly_bound_arguments.arguments.items():
    poly_parameter = polymorphic_type.parameters[name]
    if (has_var_positional and
        poly_parameter.kind is Parameter.POSITIONAL_OR_KEYWORD):
      # If there is a VAR_POSITIONAL, all POSITIONAL_OR_KEYWORD become
      # POSITIONAL_ONLY.
      parameters.append(
          _make_validated_mono_param(name, arg, Parameter.POSITIONAL_ONLY,
                                     type_context,
                                     poly_parameter.type_constraint))

    elif poly_parameter.kind is Parameter.VAR_POSITIONAL:
      # Unbundle VAR_POSITIONAL into individual POSITIONAL_ONLY args.
      for i, value in enumerate(arg):
        parameters.append(
            _make_validated_mono_param(f"{poly_parameter.name}_{i}", value,
                                       Parameter.POSITIONAL_ONLY, type_context,
                                       poly_parameter.type_constraint))

    elif poly_parameter.kind is Parameter.VAR_KEYWORD:
      # Unbundle VAR_KEYWORD into individual KEYWORD_ONLY args.
      for kwarg_name, kwarg_value in arg.items():
        parameters.append(
            _make_validated_mono_param(kwarg_name, kwarg_value,
                                       Parameter.KEYWORD_ONLY, type_context,
                                       poly_parameter.type_constraint))
    else:
      parameters.append(
          _make_validated_mono_param(name, arg, poly_parameter.kind,
                                     type_context,
                                     poly_parameter.type_constraint))

  capture_types = collections.OrderedDict()
  for name, value in captures.items():
    capture_types[name] = trace_type.from_value(value, type_context)

  monomorphic_function_type = FunctionType(parameters, capture_types)
  mono_bound_arguments = monomorphic_function_type.bind(
      *poly_bound_arguments.args, **poly_bound_arguments.kwargs)

  return mono_bound_arguments, monomorphic_function_type, type_context


# TODO(fmuham): Share code with canonicalize_to_monomorphic.
def add_type_constraints(function_type: FunctionType, input_signature: Any,
                         default_values: Dict[str, Any]):
  """Adds type constraints to a FunctionType based on the input_signature."""
  context = trace_type.InternalTracingContext(is_legacy_signature=True)
  constraints = [trace_type.from_value(c, context) for c in input_signature]
  parameters = []

  has_var_pos = any(
      p.kind is p.VAR_POSITIONAL for p in function_type.parameters.values())

  for param in function_type.parameters.values():
    # VAR_POSITIONAL does not allow POSITIONAL_OR_KEYWORD args.
    sanitized_kind = (
        param.POSITIONAL_ONLY if has_var_pos and
        param.kind is param.POSITIONAL_OR_KEYWORD else param.kind)

    if param.name == "self":
      # Type constraints do not apply on them.
      parameters.append(
          Parameter("self", sanitized_kind, param.optional, None))

    elif param.kind is param.VAR_KEYWORD:
      # Disabled when input_signature is specified.
      continue

    elif param.kind is param.VAR_POSITIONAL:
      # Convert into Positional Only args based on length of constraints.
      for i in range(len(constraints)):
        parameters.append(
            Parameter(param.name + "_" + str(i), Parameter.POSITIONAL_ONLY,
                      False, constraints.pop(0)))

    elif (param.kind in [
        param.POSITIONAL_ONLY, param.POSITIONAL_OR_KEYWORD, param.KEYWORD_ONLY
    ]):
      if constraints:
        parameters.append(
            Parameter(param.name, sanitized_kind, param.optional,
                      constraints.pop(0)))
      elif param.name in default_values:
        type_constraint = trace_type.from_value(default_values[param.name])
        parameters.append(
            Parameter(param.name, sanitized_kind, param.optional,
                      type_constraint))
      # TODO(fmuham): Add check for insufficient type constraints.

  if constraints:
    raise TypeError(
        f"input_signature contains {len(constraints)} extra type constraints."
    )

  return FunctionType(parameters)
