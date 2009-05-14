/**
 * 
 */
package org.cognitivesystems.idl;

import java.io.File;
import java.net.*;

/**
 * @author nah
 */
public class IDLJWrapper {

    private static class MyLoader extends URLClassLoader {

        /**
         * @param _urls
         */
        public MyLoader(URL[] _urls) {
            // super(_urls);
            super(_urls, null);
            // System.out.println(_urls[0]);
        }

        /* (non-Javadoc)
         * @see java.net.URLClassLoader#findResource(java.lang.String)
         */
        @Override
        public URL findResource(String _name) {
            System.out.println("fr: " + _name);
            return super.findResource(_name);
        }
        
        /*
         * (non-Javadoc)
         * 
         * @see java.net.URLClassLoader#findClass(java.lang.String)
         */
        public void findAndLoad(String _name)
                throws ClassNotFoundException, InstantiationException, IllegalAccessException {
            // TODO Auto-generated method stub
//            resolveClass(findClass(_name));
             loadClass(_name).newInstance();

            // System.out.println("resource: " + findResource(_name));
        }

    }

    /**
     * @param args
     * @throws MalformedURLException
     * @throws ClassNotFoundException
     * @throws IllegalAccessException 
     * @throws InstantiationException 
     */
    public static void main(String[] args)
            throws MalformedURLException, ClassNotFoundException, InstantiationException, IllegalAccessException {

//        System.s
        
        // URLClassLoader loader = URLClassLoader.newInstance(new
        // URL[]{new
        // File(args[0]).toURL()},Thread.currentThread().getContextClassLoader());
        MyLoader loader = new MyLoader(new URL[] {
            new File(args[0]).toURL()
        });

        
        
        loader.findAndLoad("org.cognitivesystems.idl.Compile");
        loader
            .findAndLoad("org.cognitivesystems.idl.InvalidArgument");
        loader.findAndLoad("org.cognitivesystems.idl.PragmaHandler");
        loader.findAndLoad("org.cognitivesystems.idl.NoPragma");
        loader.findAndLoad("org.cognitivesystems.idl.Generator");
        loader.findAndLoad("org.cognitivesystems.idl.AttributeGen");
        loader.findAndLoad("org.cognitivesystems.idl.ConstGen");
        loader.findAndLoad("org.cognitivesystems.idl.EnumGen");
        loader.findAndLoad("org.cognitivesystems.idl.ExceptionGen");
        loader.findAndLoad("org.cognitivesystems.idl.ForwardGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.ForwardValueGen");
        loader.findAndLoad("org.cognitivesystems.idl.IncludeGen");
        loader.findAndLoad("org.cognitivesystems.idl.InterfaceGen");
        loader.findAndLoad("org.cognitivesystems.idl.ValueGen");
        loader.findAndLoad("org.cognitivesystems.idl.ValueBoxGen");
        loader.findAndLoad("org.cognitivesystems.idl.MethodGen");
        loader.findAndLoad("org.cognitivesystems.idl.ModuleGen");
        loader.findAndLoad("org.cognitivesystems.idl.NativeGen");
        loader.findAndLoad("org.cognitivesystems.idl.ParameterGen");
        loader.findAndLoad("org.cognitivesystems.idl.PragmaGen");
        loader.findAndLoad("org.cognitivesystems.idl.PrimitiveGen");
        loader.findAndLoad("org.cognitivesystems.idl.SequenceGen");
        loader.findAndLoad("org.cognitivesystems.idl.StringGen");
        loader.findAndLoad("org.cognitivesystems.idl.StructGen");
        loader.findAndLoad("org.cognitivesystems.idl.TypedefGen");
        loader.findAndLoad("org.cognitivesystems.idl.UnionGen");
        loader.findAndLoad("org.cognitivesystems.idl.GenFactory");
        loader.findAndLoad("org.cognitivesystems.idl.Noop");
        loader.findAndLoad("org.cognitivesystems.idl.SymtabFactory");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.ExprFactory");
        loader.findAndLoad("org.cognitivesystems.idl.SymtabEntry");
        loader.findAndLoad("org.cognitivesystems.idl.ModuleEntry");
        loader.findAndLoad("org.cognitivesystems.idl.TypedefEntry");
        loader.findAndLoad("org.cognitivesystems.idl.Factories");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.Factories");
        loader.findAndLoad("org.cognitivesystems.idl.Preprocessor");
        loader.findAndLoad("org.cognitivesystems.idl.ParseException");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.Expression");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.Terminal");
        loader.findAndLoad("org.cognitivesystems.idl.PragmaEntry");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.BinaryExpr");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.BooleanOr");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.UnaryExpr");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.BooleanNot");
        loader.findAndLoad("org.cognitivesystems.idl.IncludeEntry");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.Equal");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.NotEqual");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.GreaterThan");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.GreaterEqual");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.LessThan");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.LessEqual");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.EvaluationException");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.BooleanAnd");
        loader.findAndLoad("org.cognitivesystems.idl.RepositoryID");
        loader.findAndLoad("org.cognitivesystems.idl.IDLID");
        loader.findAndLoad("org.cognitivesystems.idl.Parser");
        loader.findAndLoad("org.cognitivesystems.idl.StructEntry");
        loader.findAndLoad("org.cognitivesystems.idl.constExpr.Xor");
        loader.findAndLoad("org.cognitivesystems.idl.InterfaceType");
        loader.findAndLoad("org.cognitivesystems.idl.InterfaceEntry");
        loader.findAndLoad("org.cognitivesystems.idl.ValueEntry");
        loader.findAndLoad("org.cognitivesystems.idl.ForwardEntry");
        loader
            .findAndLoad("org.cognitivesystems.idl.ForwardValueEntry");
        loader.findAndLoad("org.cognitivesystems.idl.ValueBoxEntry");
        loader.findAndLoad("org.cognitivesystems.idl.UnionEntry");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.Positive");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.Negative");
        loader.findAndLoad("org.cognitivesystems.idl.constExpr.Not");
        loader.findAndLoad("org.cognitivesystems.idl.EnumEntry");
        loader.findAndLoad("org.cognitivesystems.idl.SequenceEntry");
        loader.findAndLoad("org.cognitivesystems.idl.StringEntry");
        loader.findAndLoad("org.cognitivesystems.idl.PrimitiveEntry");
        loader.findAndLoad("org.cognitivesystems.idl.ConstEntry");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.ShiftLeft");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.ShiftRight");
        loader.findAndLoad("org.cognitivesystems.idl.ExceptionEntry");
        loader.findAndLoad("org.cognitivesystems.idl.MethodEntry");
        loader.findAndLoad("org.cognitivesystems.idl.NativeEntry");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.Times");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.Divide");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.Modulo");
        loader.findAndLoad("org.cognitivesystems.idl.AttributeEntry");
        loader.findAndLoad("org.cognitivesystems.idl.constExpr.Plus");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.Minus");
        loader.findAndLoad("org.cognitivesystems.idl.constExpr.Or");
        loader.findAndLoad("org.cognitivesystems.idl.constExpr.And");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.AuxGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.Helper");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.Helper24");
        loader.findAndLoad("org.cognitivesystems.idl.Arguments");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.Arguments");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.Holder");
        loader.findAndLoad("org.cognitivesystems.idl.GenFileStream");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.Skeleton");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.Stub");
        loader
            .findAndLoad("org.cognitivesystems.idl.DefaultSymtabFactory");
        loader.findAndLoad("org.cognitivesystems.idl.Util");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.Util");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.NameModifier");
        loader
            .findAndLoad("org.cognitivesystems.idl.constExpr.DefaultExprFactory");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.GenFactory");
        loader
            .findAndLoad("org.cognitivesystems.idl.som.cff.FileLocator");
        loader
            .findAndLoad("org.cognitivesystems.idl.som.cff.NamedDataInputStream");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.MethodGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.AttributeGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.MethodGenClone24");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.AttributeGen24");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.ConstGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.JavaGenerator");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.EnumGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.StructGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.ExceptionGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.InterfaceGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.ValueGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.ValueGen24");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.ValueBoxGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.ValueBoxGen24");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.MethodGen24");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.ModuleGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.NativeGen");
        loader.findAndLoad("org.cognitivesystems.idl.ParameterEntry");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.PrimitiveGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.SequenceGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.StringGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.TypedefGen");
        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.UnionGen");
        loader.findAndLoad("org.cognitivesystems.idl.TokenBuffer");
        loader.findAndLoad("org.cognitivesystems.idl.Token");
        loader.findAndLoad("org.cognitivesystems.idl.Scanner");
        loader
            .findAndLoad("org.cognitivesystems.idl.InvalidCharacter");
        loader.findAndLoad("org.cognitivesystems.idl.ScannerData");
        loader.findAndLoad("org.cognitivesystems.idl.Comment");

        loader
            .findAndLoad("org.cognitivesystems.idl.toJavaPortable.Compile");

        // loader.

        // System.out.println(args[0]);
        org.cognitivesystems.idl.toJavaPortable.Compile.main(args);
    }

}
